/*
 * GyroMan cascaded PID firmware
 *
 * This is a clean ESP32 + MPU6050 + brushed DC motor balancing baseline.
 * It intentionally keeps tuning to four controller gains. Hardware-specific
 * calibration, including encoder-aware actuator compensation, stays fixed in
 * firmware.
 *
 * Open-source references used for architecture comparison:
 * - 456789123/Balancing_Robot_ESP32:
 *   ESP32 + MPU6050, simple angle PID, gyro-rate D term, direct PWM.
 * - hamzanaeemj/self-balancing-robot:
 *   ESP32 + MPU6050 + GA25-370 encoders, cascaded angle/velocity PID.
 * - Rokasbarasa1/ESP32-self-balancing-robot:
 *   ESP32 + MPU6050 + TB6612 brushed motors; notes that motor offset
 *   compensation made the robot jitterier and was removed.
 *
 * The implementation below is written for Simon's actual wiring. Only four
 * controller gains are user-tunable: upright P/D and velocity P/I. Calibration,
 * actuator limits, and motor linearization stay fixed in firmware.
 */

#include <Arduino.h>
#include <Wire.h>

#include "hardware_config.h"
#include "kalman_1d.h"

namespace control {

constexpr char FIRMWARE_VERSION[] = "0823test pass";

// Fast upright loop plus a slower encoder velocity loop.
constexpr uint32_t CONTROL_PERIOD_US = 3000;
constexpr uint32_t SPEED_PERIOD_MS = 20;
constexpr uint32_t LOG_PERIOD_MS = 20;

// The only four tuning gains:
//   velocity PI -> target angle; upright PD -> motor effort.
constexpr float ANGLE_KP = 23.1f;  // PWM/deg
constexpr float ANGLE_KD = 1.18f;   // PWM/(deg/s)
constexpr float SPEED_KP = 10.0f;   // deg/(wheel rev/s)
constexpr float SPEED_KI = 10.0f;   // deg/wheel rev
constexpr float TARGET_ANGLE_LIMIT_DEG = 6.0f;

// Fixed hardware calibration and actuator authority.
constexpr float BALANCE_TRIM_DEG = -3.25f;
constexpr int MAX_MOTOR_PWM = 255;
constexpr int PWM_ZERO_BAND = 1;
constexpr int LEFT_FORWARD_START_PWM = 145;
constexpr int LEFT_BACKWARD_START_PWM = 125;
constexpr int RIGHT_FORWARD_START_PWM = 138;
constexpr int RIGHT_BACKWARD_START_PWM = 123;
constexpr float ACTUATOR_PDM_PRECHARGE = 1.0f;

// Fixed left/right wheel matching. The two motors keep the same logical command,
// then encoder feedback trims only the difference: left ahead -> less left PWM
// and more right PWM. This keeps tuning focused on the four balance gains.
constexpr float WHEEL_MATCH_SPEED_KP = 60.0f;      // PWM/(rev/s difference)
constexpr float WHEEL_MATCH_POSITION_KP = 120.0f;  // PWM/rev difference
constexpr float WHEEL_MATCH_MAX_PWM = 55.0f;

constexpr float AUTO_START_ANGLE_DEG = 5.0f;
constexpr float AUTO_START_RATE_DPS = 12.0f;
constexpr uint32_t AUTO_START_HOLD_MS = 120;
constexpr float FALL_ANGLE_DEG = 35.0f;
constexpr uint32_t FALL_RECOVERY_MS = 700;

}  // namespace control

enum class RobotState : uint8_t {
  CALIBRATING,
  WAIT_UPRIGHT,
  BALANCING,
  RECOVERY,
  FAULT,
};

struct ImuRaw {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

struct MotorPins {
  uint8_t in1;
  uint8_t in2;
  bool reversed;
};

struct ActuatorState {
  float boostPwm = 0.0f;
  float pulseAccumulator = 0.0f;
  int8_t commandSign = 0;
};

const MotorPins LEFT_MOTOR = {
    hw::LEFT_MOTOR_IN1, hw::LEFT_MOTOR_IN2, hw::LEFT_MOTOR_REVERSED};
const MotorPins RIGHT_MOTOR = {
    hw::RIGHT_MOTOR_IN1, hw::RIGHT_MOTOR_IN2, hw::RIGHT_MOTOR_REVERSED};

static const int8_t DRAM_ATTR QUADRATURE_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t leftRawCount = 0;
volatile int32_t rightRawCount = 0;
volatile uint8_t leftEncoderState = 0;
volatile uint8_t rightEncoderState = 0;

Kalman1D pitchFilter;
RobotState robotState = RobotState::CALIBRATING;

float gyroBiasY = 0.0f;
float pitchDeg = 0.0f;
float gyroRateDps = 0.0f;
float leftWheelPositionRev = 0.0f;
float rightWheelPositionRev = 0.0f;
float wheelPositionRev = 0.0f;
float targetPositionRev = 0.0f;
float positionErrorRev = 0.0f;
float leftWheelSpeedRevS = 0.0f;
float rightWheelSpeedRevS = 0.0f;
float wheelSpeedRevS = 0.0f;
float speedUpdateDtS = 0.0f;
float wheelPositionDiffRev = 0.0f;
float wheelSpeedDiffRevS = 0.0f;
float lastLeftSpeedPositionRev = 0.0f;
float lastRightSpeedPositionRev = 0.0f;
float lastSpeedPositionRev = 0.0f;
float targetSpeedRevS = 0.0f;
float speedErrorRevS = 0.0f;
float speedIntegralRev = 0.0f;
float speedPContributionDeg = 0.0f;
float speedIContributionDeg = 0.0f;
float targetAngleOffsetDeg = 0.0f;
float angleTargetDeg = control::BALANCE_TRIM_DEG;
float angleErrorDeg = 0.0f;
float anglePContributionPwm = 0.0f;
float angleDContributionPwm = 0.0f;
float wheelMatchFeedbackPwm = 0.0f;
float leftActuatorBoostPwm = 0.0f;
float rightActuatorBoostPwm = 0.0f;

int currentLeftIdealPwm = 0;
int currentRightIdealPwm = 0;
int currentLeftPwm = 0;
int currentRightPwm = 0;
int currentIdealPwm = 0;
ActuatorState leftActuator;
ActuatorState rightActuator;
uint32_t nextControlUs = 0;
uint32_t lastControlUs = 0;
uint32_t lastSpeedUpdateMs = 0;
uint32_t lastLogMs = 0;
uint32_t uprightSinceMs = 0;
uint32_t recoveryUntilMs = 0;
uint8_t consecutiveImuFailures = 0;

const char* stateName(RobotState state) {
  switch (state) {
    case RobotState::CALIBRATING: return "CALIBRATING";
    case RobotState::WAIT_UPRIGHT: return "WAIT_UPRIGHT";
    case RobotState::BALANCING: return "BALANCING";
    case RobotState::RECOVERY: return "RECOVERY";
    case RobotState::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

char stateCode(RobotState state) {
  switch (state) {
    case RobotState::CALIBRATING: return 'C';
    case RobotState::WAIT_UPRIGHT: return 'W';
    case RobotState::BALANCING: return 'B';
    case RobotState::RECOVERY: return 'R';
    case RobotState::FAULT: return 'F';
  }
  return 'U';
}

uint8_t readEncoderPair(uint8_t pinA, uint8_t pinB) {
  return (digitalRead(pinA) << 1) | digitalRead(pinB);
}

void IRAM_ATTR onLeftEncoder() {
  const uint8_t next =
      (digitalRead(hw::LEFT_ENCODER_A) << 1) | digitalRead(hw::LEFT_ENCODER_B);
  portENTER_CRITICAL_ISR(&encoderMux);
  leftRawCount += QUADRATURE_TABLE[(leftEncoderState << 2) | next];
  leftEncoderState = next;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR onRightEncoder() {
  const uint8_t next =
      (digitalRead(hw::RIGHT_ENCODER_A) << 1) | digitalRead(hw::RIGHT_ENCODER_B);
  portENTER_CRITICAL_ISR(&encoderMux);
  rightRawCount += QUADRATURE_TABLE[(rightEncoderState << 2) | next];
  rightEncoderState = next;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void readEncoderCounts(int32_t& left, int32_t& right) {
  portENTER_CRITICAL(&encoderMux);
  left = leftRawCount;
  right = rightRawCount;
  portEXIT_CRITICAL(&encoderMux);
  if (hw::LEFT_ENCODER_REVERSED) left = -left;
  if (hw::RIGHT_ENCODER_REVERSED) right = -right;
}

void zeroEncoders() {
  portENTER_CRITICAL(&encoderMux);
  leftRawCount = 0;
  rightRawCount = 0;
  leftEncoderState = readEncoderPair(hw::LEFT_ENCODER_A, hw::LEFT_ENCODER_B);
  rightEncoderState = readEncoderPair(hw::RIGHT_ENCODER_A, hw::RIGHT_ENCODER_B);
  portEXIT_CRITICAL(&encoderMux);
}

void initEncoders() {
  pinMode(hw::LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(hw::LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(hw::RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(hw::RIGHT_ENCODER_B, INPUT_PULLUP);
  zeroEncoders();
  attachInterrupt(digitalPinToInterrupt(hw::LEFT_ENCODER_A), onLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hw::LEFT_ENCODER_B), onLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hw::RIGHT_ENCODER_A), onRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hw::RIGHT_ENCODER_B), onRightEncoder, CHANGE);
}

void writeMotorDuty(uint8_t pin, int duty) {
  ledcWrite(pin, constrain(duty, 0, 255));
}

void resetActuators() {
  leftActuator = ActuatorState();
  rightActuator = ActuatorState();
  leftActuatorBoostPwm = 0.0f;
  rightActuatorBoostPwm = 0.0f;
}

void stopMotors() {
  writeMotorDuty(hw::LEFT_MOTOR_IN1, 0);
  writeMotorDuty(hw::LEFT_MOTOR_IN2, 0);
  writeMotorDuty(hw::RIGHT_MOTOR_IN1, 0);
  writeMotorDuty(hw::RIGHT_MOTOR_IN2, 0);
  currentLeftPwm = 0;
  currentRightPwm = 0;
  currentLeftIdealPwm = 0;
  currentRightIdealPwm = 0;
  currentIdealPwm = 0;
  resetActuators();
}

int motorStartPwm(const MotorPins& motor, bool logicalForward) {
  const bool isLeft = motor.in1 == hw::LEFT_MOTOR_IN1;
  if (isLeft) {
    return logicalForward ? control::LEFT_FORWARD_START_PWM
                          : control::LEFT_BACKWARD_START_PWM;
  }
  return logicalForward ? control::RIGHT_FORWARD_START_PWM
                        : control::RIGHT_BACKWARD_START_PWM;
}

float moveToward(float value, float target, float maxStep) {
  if (value < target) return fminf(value + maxStep, target);
  if (value > target) return fmaxf(value - maxStep, target);
  return value;
}

int actuatorMotorCommand(const MotorPins& motor,
                         int signedEffort,
                         float wheelSpeedRevS,
                         ActuatorState& actuator,
                         float& boostOut) {
  signedEffort =
      constrain(signedEffort, -control::MAX_MOTOR_PWM, control::MAX_MOTOR_PWM);
  const int magnitude = abs(signedEffort);
  if (magnitude <= control::PWM_ZERO_BAND) {
    actuator = ActuatorState();
    boostOut = 0.0f;
    return 0;
  }

  const int8_t commandSign = signedEffort > 0 ? 1 : -1;
  if (commandSign != actuator.commandSign) {
    actuator.boostPwm = 0.0f;
    actuator.pulseAccumulator = control::ACTUATOR_PDM_PRECHARGE;
    actuator.commandSign = commandSign;
  }

  const int startPwm =
      constrain(motorStartPwm(motor, signedEffort > 0),
                control::PWM_ZERO_BAND + 1,
                control::MAX_MOTOR_PWM);
  (void)wheelSpeedRevS;
  int outputMagnitude = magnitude;
  if (magnitude < startPwm) {
    const float density = constrain(float(magnitude) / float(startPwm),
                                    0.0f,
                                    1.0f);
    actuator.pulseAccumulator += density;
    if (actuator.pulseAccumulator >= 1.0f) {
      actuator.pulseAccumulator -= 1.0f;
      outputMagnitude = startPwm;
    } else {
      outputMagnitude = 0;
    }
  }
  actuator.boostPwm = float(outputMagnitude - magnitude);
  boostOut = actuator.boostPwm;

  outputMagnitude = constrain(outputMagnitude, 0, control::MAX_MOTOR_PWM);
  return commandSign > 0 ? outputMagnitude : -outputMagnitude;
}

void setMotor(const MotorPins& motor, int signedPwm) {
  signedPwm = constrain(signedPwm, -control::MAX_MOTOR_PWM, control::MAX_MOTOR_PWM);
  if (motor.reversed) signedPwm = -signedPwm;

  writeMotorDuty(motor.in1, 0);
  writeMotorDuty(motor.in2, 0);
  if (signedPwm > 0) {
    writeMotorDuty(motor.in1, signedPwm);
  } else if (signedPwm < 0) {
    writeMotorDuty(motor.in2, -signedPwm);
  }
}

void driveBoth(float commandPwm, float wheelMatchPwm = 0.0f) {
  const int command = lroundf(constrain(commandPwm,
                                        -control::MAX_MOTOR_PWM,
                                        control::MAX_MOTOR_PWM));
  const float diffLimit =
      fminf(control::WHEEL_MATCH_MAX_PWM, fmaxf(8.0f, fabsf(command) * 0.6f));
  const int match = lroundf(constrain(wheelMatchPwm, -diffLimit, diffLimit));
  const int leftCommand = constrain(command - match,
                                    -control::MAX_MOTOR_PWM,
                                    control::MAX_MOTOR_PWM);
  const int rightCommand = constrain(command + match,
                                     -control::MAX_MOTOR_PWM,
                                     control::MAX_MOTOR_PWM);
  currentIdealPwm = command;
  currentLeftIdealPwm = leftCommand;
  currentRightIdealPwm = rightCommand;

  currentLeftPwm =
      actuatorMotorCommand(LEFT_MOTOR, leftCommand, leftWheelSpeedRevS,
                           leftActuator, leftActuatorBoostPwm);
  currentRightPwm =
      actuatorMotorCommand(RIGHT_MOTOR, rightCommand, rightWheelSpeedRevS,
                           rightActuator, rightActuatorBoostPwm);

  setMotor(LEFT_MOTOR, currentLeftPwm);
  setMotor(RIGHT_MOTOR, currentRightPwm);
}

bool initMotorOutputs() {
  const uint8_t pins[] = {
      hw::LEFT_MOTOR_IN1, hw::LEFT_MOTOR_IN2,
      hw::RIGHT_MOTOR_IN1, hw::RIGHT_MOTOR_IN2};
  bool ok = true;
  for (uint8_t pin : pins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    ok = ledcAttach(pin, hw::PWM_FREQUENCY_HZ, hw::PWM_RESOLUTION_BITS) && ok;
    ledcWrite(pin, 0);
  }
  stopMotors();
  return ok;
}

bool writeImuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(hw::IMU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

int16_t readI16() {
  const uint8_t high = Wire.read();
  const uint8_t low = Wire.read();
  return int16_t((uint16_t(high) << 8) | low);
}

bool readImuRaw(ImuRaw& raw) {
  Wire.beginTransmission(hw::IMU_ADDRESS);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(hw::IMU_ADDRESS, uint8_t(14)) != 14) return false;

  raw.ax = readI16();
  raw.ay = readI16();
  raw.az = readI16();
  readI16();
  raw.gx = readI16();
  raw.gy = readI16();
  raw.gz = readI16();
  return true;
}

float calculateAccelPitch(const ImuRaw& raw) {
  const float ax = raw.ax;
  const float ay = raw.ay;
  const float az = raw.az;
  return atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

bool initImu() {
  Wire.end();
  Wire.begin(hw::IMU_SDA, hw::IMU_SCL, hw::I2C_FREQUENCY_HZ);
  Wire.beginTransmission(hw::IMU_ADDRESS);
  if (Wire.endTransmission() != 0) return false;

  if (!writeImuRegister(0x6B, 0x80)) return false;
  delay(100);
  if (!writeImuRegister(0x6B, 0x01)) return false;
  return writeImuRegister(0x19, 0x04) &&
         writeImuRegister(0x1A, 0x01) &&
         writeImuRegister(0x1B, 0x00) &&
         writeImuRegister(0x1C, 0x00);
}

bool calibrateGyro() {
  robotState = RobotState::CALIBRATING;
  stopMotors();
  Serial.println("calibration: keep robot still for 2 seconds");

  constexpr int samplesNeeded = 600;
  double sum = 0.0;
  double sumSquares = 0.0;
  int samples = 0;
  ImuRaw raw{};
  for (int i = 0; i < samplesNeeded + 80; ++i) {
    if (readImuRaw(raw)) {
      const float gyroY = raw.gy / 131.0f;
      sum += gyroY;
      sumSquares += gyroY * gyroY;
      samples++;
      if (samples >= samplesNeeded) break;
    }
    delayMicroseconds(control::CONTROL_PERIOD_US);
  }

  if (samples < samplesNeeded * 9 / 10) return false;
  gyroBiasY = sum / samples;
  const float variance = fmaxf(0.0f, sumSquares / samples - gyroBiasY * gyroBiasY);
  if (sqrtf(variance) > 3.0f) {
    Serial.println("calibration rejected: robot moved");
    return false;
  }

  if (!readImuRaw(raw)) return false;
  const float accelPitch = calculateAccelPitch(raw);
  pitchFilter = Kalman1D();
  pitchFilter.setAngle(accelPitch);
  pitchDeg = accelPitch;
  gyroRateDps = 0.0f;
  consecutiveImuFailures = 0;
  Serial.printf("calibration complete: gyro_bias_y=%.4f, pitch=%.3f\n",
                gyroBiasY, pitchDeg);
  return true;
}

bool updateAttitude(float dt) {
  ImuRaw raw{};
  if (!readImuRaw(raw)) {
    consecutiveImuFailures++;
    return false;
  }
  consecutiveImuFailures = 0;
  const float accelPitch = calculateAccelPitch(raw);
  const float measuredGyroY = raw.gy / 131.0f - gyroBiasY;
  pitchDeg = pitchFilter.update(accelPitch, measuredGyroY, dt);
  gyroRateDps = pitchFilter.rate();
  return true;
}

bool updateWheelState(bool reset = false) {
  int32_t left = 0;
  int32_t right = 0;
  readEncoderCounts(left, right);
  leftWheelPositionRev =
      static_cast<float>(left) / hw::ENCODER_COUNTS_PER_WHEEL_REV;
  rightWheelPositionRev =
      static_cast<float>(right) / hw::ENCODER_COUNTS_PER_WHEEL_REV;
  wheelPositionRev = (leftWheelPositionRev + rightWheelPositionRev) * 0.5f;
  wheelPositionDiffRev = leftWheelPositionRev - rightWheelPositionRev;
  positionErrorRev = wheelPositionRev - targetPositionRev;

  const uint32_t now = millis();
  if (reset) {
    lastLeftSpeedPositionRev = leftWheelPositionRev;
    lastRightSpeedPositionRev = rightWheelPositionRev;
    lastSpeedPositionRev = wheelPositionRev;
    leftWheelSpeedRevS = 0.0f;
    rightWheelSpeedRevS = 0.0f;
    wheelSpeedRevS = 0.0f;
    wheelSpeedDiffRevS = 0.0f;
    speedUpdateDtS = 0.0f;
    lastSpeedUpdateMs = now;
    return true;
  }
  const uint32_t elapsedMs = now - lastSpeedUpdateMs;
  if (elapsedMs >= control::SPEED_PERIOD_MS) {
    speedUpdateDtS = elapsedMs * 0.001f;
    leftWheelSpeedRevS =
        (leftWheelPositionRev - lastLeftSpeedPositionRev) * 1000.0f / elapsedMs;
    rightWheelSpeedRevS =
        (rightWheelPositionRev - lastRightSpeedPositionRev) * 1000.0f / elapsedMs;
    wheelSpeedRevS =
        (wheelPositionRev - lastSpeedPositionRev) * 1000.0f / elapsedMs;
    wheelSpeedDiffRevS = leftWheelSpeedRevS - rightWheelSpeedRevS;
    lastLeftSpeedPositionRev = leftWheelPositionRev;
    lastRightSpeedPositionRev = rightWheelPositionRev;
    lastSpeedPositionRev = wheelPositionRev;
    lastSpeedUpdateMs = now;
    return true;
  }
  return false;
}

void resetController() {
  positionErrorRev = 0.0f;
  targetSpeedRevS = 0.0f;
  speedErrorRevS = 0.0f;
  speedIntegralRev = 0.0f;
  speedPContributionDeg = 0.0f;
  speedIContributionDeg = 0.0f;
  targetAngleOffsetDeg = 0.0f;
  angleTargetDeg = control::BALANCE_TRIM_DEG;
  angleErrorDeg = 0.0f;
  anglePContributionPwm = 0.0f;
  angleDContributionPwm = 0.0f;
  wheelMatchFeedbackPwm = 0.0f;
  currentLeftIdealPwm = 0;
  currentRightIdealPwm = 0;
  currentLeftPwm = 0;
  currentRightPwm = 0;
  resetActuators();
}

void enterWaitUpright(const char* reason) {
  stopMotors();
  resetController();
  robotState = RobotState::WAIT_UPRIGHT;
  uprightSinceMs = 0;
  lastControlUs = micros();
  nextControlUs = lastControlUs + control::CONTROL_PERIOD_US;
  Serial.printf("state=WAIT_UPRIGHT reason=%s\n", reason);
}

void enterBalancing() {
  zeroEncoders();
  updateWheelState(true);
  targetPositionRev = wheelPositionRev;
  resetController();
  robotState = RobotState::BALANCING;
  nextControlUs = micros() + control::CONTROL_PERIOD_US;
  lastControlUs = micros();
  Serial.println("state=BALANCING cascaded PID active");
}

void enterRecovery(const char* reason) {
  stopMotors();
  resetController();
  robotState = RobotState::RECOVERY;
  recoveryUntilMs = millis() + control::FALL_RECOVERY_MS;
  Serial.printf("state=RECOVERY reason=%s\n", reason);
}

void updateSpeedLoop(float dt) {
  if (dt <= 0.0f) return;
  positionErrorRev = wheelPositionRev - targetPositionRev;
  speedErrorRevS = targetSpeedRevS - wheelSpeedRevS;
  speedPContributionDeg = control::SPEED_KP * speedErrorRevS;

  const float candidateIntegral = speedIntegralRev + speedErrorRevS * dt;
  const float candidateTarget =
      speedPContributionDeg + control::SPEED_KI * candidateIntegral;
  const bool insideLimit =
      fabsf(candidateTarget) <= control::TARGET_ANGLE_LIMIT_DEG;
  const bool unwindingHigh =
      candidateTarget > control::TARGET_ANGLE_LIMIT_DEG && speedErrorRevS < 0.0f;
  const bool unwindingLow =
      candidateTarget < -control::TARGET_ANGLE_LIMIT_DEG && speedErrorRevS > 0.0f;
  if (insideLimit || unwindingHigh || unwindingLow) {
    speedIntegralRev = candidateIntegral;
  }

  speedIContributionDeg = control::SPEED_KI * speedIntegralRev;
  targetAngleOffsetDeg = constrain(
      speedPContributionDeg + speedIContributionDeg,
      -control::TARGET_ANGLE_LIMIT_DEG,
      control::TARGET_ANGLE_LIMIT_DEG);
}

float updateAngleLoop() {
  angleTargetDeg = control::BALANCE_TRIM_DEG + targetAngleOffsetDeg;
  angleErrorDeg = pitchDeg - angleTargetDeg;
  anglePContributionPwm = control::ANGLE_KP * angleErrorDeg;
  angleDContributionPwm = control::ANGLE_KD * gyroRateDps;
  wheelMatchFeedbackPwm = constrain(
      control::WHEEL_MATCH_POSITION_KP * wheelPositionDiffRev +
          control::WHEEL_MATCH_SPEED_KP * wheelSpeedDiffRevS,
      -control::WHEEL_MATCH_MAX_PWM,
      control::WHEEL_MATCH_MAX_PWM);
  return constrain(anglePContributionPwm + angleDContributionPwm,
                   -control::MAX_MOTOR_PWM,
                   control::MAX_MOTOR_PWM);
}

void logTelemetry() {
  const uint32_t now = millis();
  if (now - lastLogMs < control::LOG_PERIOD_MS) return;
  lastLogMs = now;
  Serial.printf(
      "tel,s=%c,a=%.2f,g=%.1f,t=%.2f,ta=%.2f,ae=%.2f,tv=%.2f,v=%.3f,ve=%.3f,ix=%.3f,p=%.3f,ld=%.3f,sd=%.3f,u=%d,li=%d,ri=%d,lp=%d,rp=%d,b=%.0f/%.0f,ap=%.0f,ad=%.0f,sp=%.2f,si=%.2f,m=%.0f\n",
      stateCode(robotState),
      pitchDeg,
      gyroRateDps,
      control::BALANCE_TRIM_DEG,
      angleTargetDeg,
      angleErrorDeg,
      targetSpeedRevS,
      wheelSpeedRevS,
      speedErrorRevS,
      speedIntegralRev,
      positionErrorRev,
      wheelPositionDiffRev,
      wheelSpeedDiffRevS,
      currentIdealPwm,
      currentLeftIdealPwm,
      currentRightIdealPwm,
      currentLeftPwm,
      currentRightPwm,
      leftActuatorBoostPwm,
      rightActuatorBoostPwm,
      anglePContributionPwm,
      angleDContributionPwm,
      speedPContributionDeg,
      speedIContributionDeg,
      wheelMatchFeedbackPwm);
}

void printConfig() {
  Serial.println();
  Serial.printf("firmware=%s\n", control::FIRMWARE_VERSION);
  Serial.println("wiring=mpu SDA21/SCL22, motors L25/26 R27/14, encoders L32/33 R18/19");
  Serial.printf("upright_pd: kp=%.3f kd=%.3f\n",
                control::ANGLE_KP, control::ANGLE_KD);
  Serial.printf("velocity_pi: kp=%.3f ki=%.3f target_angle_limit=%.2fdeg\n",
                control::SPEED_KP, control::SPEED_KI,
                control::TARGET_ANGLE_LIMIT_DEG);
  Serial.printf("cascade: angle_kp=%.3f angle_kd=%.3f speed_kp=%.3f speed_ki=%.3f\n",
                control::ANGLE_KP, control::ANGLE_KD,
                control::SPEED_KP, control::SPEED_KI);
  Serial.println("control_law: speed PI -> target angle; upright PD -> PWM");
  Serial.printf("loop: control=%luus speed=%lums imu_dlpf=0x01\n",
                static_cast<unsigned long>(control::CONTROL_PERIOD_US),
                static_cast<unsigned long>(control::SPEED_PERIOD_MS));
  Serial.println("wheel_match: left/right encoder difference trims split PWM");
  Serial.printf("fixed: trim=%.3fdeg max_pwm=%d zero_band=%d\n",
                control::BALANCE_TRIM_DEG, control::MAX_MOTOR_PWM,
                control::PWM_ZERO_BAND);
  Serial.printf("motor_static_friction_map LF/LB/RF/RB=%d/%d/%d/%d\n",
                control::LEFT_FORWARD_START_PWM,
                control::LEFT_BACKWARD_START_PWM,
                control::RIGHT_FORWARD_START_PWM,
                control::RIGHT_BACKWARD_START_PWM);
  Serial.printf("static_friction_pdm: precharge=%.2f, sub-deadzone effort becomes start-pwm pulses\n",
                control::ACTUATOR_PDM_PRECHARGE);
  Serial.printf("wheel_match_fixed: speed_kp=%.3f position_kp=%.3f max=%.1f\n",
                control::WHEEL_MATCH_SPEED_KP,
                control::WHEEL_MATCH_POSITION_KP,
                control::WHEEL_MATCH_MAX_PWM);
  Serial.println();
}

void setup() {
  Serial.begin(hw::SERIAL_BAUD);
  delay(300);
  printConfig();

  if (!initMotorOutputs()) {
    robotState = RobotState::FAULT;
    Serial.println("fault: pwm init failed");
    return;
  }
  initEncoders();

  if (!initImu()) {
    robotState = RobotState::FAULT;
    Serial.println("fault: mpu6050 not found");
    return;
  }
  while (!calibrateGyro()) {
    delay(500);
  }
  enterWaitUpright("boot");
}

void loop() {
  if (robotState == RobotState::FAULT) {
    stopMotors();
    delay(250);
    return;
  }

  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextControlUs) < 0) {
    delayMicroseconds(200);
    return;
  }
  nextControlUs += control::CONTROL_PERIOD_US;
  if ((int32_t)(nowUs - nextControlUs) > (int32_t)control::CONTROL_PERIOD_US) {
    nextControlUs = nowUs + control::CONTROL_PERIOD_US;
  }

  float dt = (nowUs - lastControlUs) * 0.000001f;
  dt = constrain(dt, 0.001f, 0.030f);
  lastControlUs = nowUs;

  if (!updateAttitude(dt)) {
    stopMotors();
    if (consecutiveImuFailures >= 5) {
      robotState = RobotState::FAULT;
      Serial.println("fault: imu read failed repeatedly");
    }
    return;
  }
  const bool speedUpdated = updateWheelState();

  switch (robotState) {
    case RobotState::CALIBRATING:
      stopMotors();
      break;

    case RobotState::WAIT_UPRIGHT:
      stopMotors();
      if (fabsf(pitchDeg - control::BALANCE_TRIM_DEG) <=
              control::AUTO_START_ANGLE_DEG &&
          fabsf(gyroRateDps) <= control::AUTO_START_RATE_DPS) {
        if (uprightSinceMs == 0) uprightSinceMs = millis();
        if (millis() - uprightSinceMs >= control::AUTO_START_HOLD_MS) {
          enterBalancing();
        }
      } else {
        uprightSinceMs = 0;
      }
      break;

    case RobotState::BALANCING:
      if (fabsf(pitchDeg - control::BALANCE_TRIM_DEG) > control::FALL_ANGLE_DEG) {
        enterRecovery("angle cutoff");
        break;
      }
      if (speedUpdated) updateSpeedLoop(speedUpdateDtS);
      driveBoth(updateAngleLoop(), wheelMatchFeedbackPwm);
      break;

    case RobotState::RECOVERY:
      stopMotors();
      if ((int32_t)(millis() - recoveryUntilMs) >= 0) {
        enterWaitUpright("recovery complete");
      }
      break;

    case RobotState::FAULT:
      stopMotors();
      break;
  }

  logTelemetry();
}
