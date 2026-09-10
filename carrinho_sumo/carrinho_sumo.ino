/*
  carrinho_sumo.ino

  Robô de sumô com ESP32 DevKit V1, dois módulos HW-039/BTS7960,
  cinco sensores digitais de borda e dois HC-SR04.

  Compatível com Arduino-ESP32 Core 3.x.
  Controle totalmente não bloqueante: millis() e micros().
*/

#include <Arduino.h>
#include "esp_random.h"

// ============================================================================
// 1. CONFIGURAÇÃO DE PINOS
// ============================================================================

// HW-039 do motor esquerdo: R_EN e L_EN ligados diretamente a 3,3 V.
constexpr uint8_t PIN_MOTOR_LEFT_RPWM = 27;
constexpr uint8_t PIN_MOTOR_LEFT_LPWM = 26;

// HW-039 do motor direito: R_EN e L_EN ligados diretamente a 3,3 V.
constexpr uint8_t PIN_MOTOR_RIGHT_RPWM = 25;
constexpr uint8_t PIN_MOTOR_RIGHT_LPWM = 33;

// A pinagem de ambos os motores já está organizada nos sentidos frente/ré.
constexpr bool INVERT_LEFT_MOTOR = false;
constexpr bool INVERT_RIGHT_MOTOR = false;

// Sensores ultrassônicos.
constexpr uint8_t PIN_HCSR04_LEFT_TRIG = 5;
constexpr uint8_t PIN_HCSR04_LEFT_ECHO = 18;
constexpr uint8_t PIN_HCSR04_RIGHT_TRIG = 19;
constexpr uint8_t PIN_HCSR04_RIGHT_ECHO = 23;

// Sensores TCRT traseiros.
constexpr uint8_t PIN_EDGE_REAR_LEFT = 13;
constexpr uint8_t PIN_EDGE_REAR_RIGHT = 32;

// TCRT5000 frontal de três canais.
constexpr uint8_t PIN_EDGE_FRONT_LEFT = 34;
constexpr uint8_t PIN_EDGE_FRONT_CENTER = 35;
constexpr uint8_t PIN_EDGE_FRONT_RIGHT = 14;

// ============================================================================
// 2. CONSTANTES CALIBRÁVEIS
// ============================================================================

constexpr bool DEBUG = true;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;
constexpr uint32_t DEBUG_INTERVAL_MS = 250;

// Os sensores frontais e traseiros usam níveis ativos diferentes.
constexpr int EDGE_FRONT_DETECTED_LEVEL = LOW;
constexpr int EDGE_REAR_DETECTED_LEVEL = HIGH;

// PWM: domínio lógico -255..255, limitado fisicamente a 0..180.
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr uint8_t PWM_PHYSICAL_MAX = 180;
constexpr uint32_t MOTOR_REVERSAL_DEADTIME_US = 3000;

// Atraso inicial aleatório inclusivo: 0 a 3000 ms.
constexpr uint32_t STARTUP_DELAY_MAX_MS = 3000;

// Velocidades lógicas. O driver recebe no máximo PWM_PHYSICAL_MAX.
constexpr int INITIAL_ADVANCE_SPEED = 155;
constexpr int SEARCH_SPEED = 160;
constexpr int TURN_SPEED = 170;
constexpr int ATTACK_SPEED = 255;
constexpr int ATTACK_TURN_SPEED = 190;
constexpr int EDGE_ESCAPE_SPEED = 180;

// Tempos de giro aproximados; precisam ser calibrados no robô real.
constexpr uint32_t TURN_90_MS = 350;
constexpr uint32_t TURN_180_MS = 700;

// Recuperação de borda.
constexpr uint32_t EDGE_IMMEDIATE_STOP_MS = 60;
constexpr uint32_t EDGE_ESCAPE_MS = 350;
constexpr uint32_t EDGE_SETTLE_MS = 50;
constexpr uint32_t EDGE_RECOVERY_TURN_MS = 300;
constexpr uint32_t EDGE_BOTH_TURN_MS = 180;
constexpr uint32_t EDGE_VERIFY_MS = 70;
constexpr uint8_t EDGE_RECOVERY_MAX_ATTEMPTS = 30;

// Ultrassônicos.
constexpr uint16_t OPPONENT_DISTANCE_CM = 70;
constexpr uint16_t ULTRASONIC_MIN_VALID_CM = 2;
constexpr uint16_t ULTRASONIC_MAX_VALID_CM = 200;
constexpr uint32_t ULTRASONIC_INTER_SENSOR_US = 30000;
constexpr uint32_t ULTRASONIC_TRIGGER_LOW_US = 3;
constexpr uint32_t ULTRASONIC_TRIGGER_HIGH_US = 10;
constexpr uint32_t ULTRASONIC_ECHO_TIMEOUT_US = 15000;
constexpr uint32_t ULTRASONIC_READING_MAX_AGE_MS = 180;

// Tolerância para perda momentânea do alvo.
constexpr uint32_t TARGET_LOST_MS = 300;

// ============================================================================
// 3. TIPOS E ESTRUTURAS
// ============================================================================

enum class RobotState : uint8_t {
  STARTUP_DELAY,
  INITIAL_ADVANCE,
  EDGE_RECOVERY,
  SEARCH_TURN_RIGHT,
  SEARCH_TURN_LEFT,
  SEARCH_FACE_BACK,
  ATTACK,
  TARGET_LOST,
  SAFE_STOP
};

enum class EdgeCategory : uint8_t {
  NONE,
  FRONT,
  REAR,
  BOTH
};

enum class RecoveryPhase : uint8_t {
  STOPPING,
  ESCAPING,
  SETTLING,
  TURNING,
  VERIFYING
};

enum class UltrasonicSide : uint8_t {
  LEFT,
  RIGHT
};

enum class UltrasonicPhase : uint8_t {
  IDLE,
  TRIGGER_LOW,
  TRIGGER_HIGH,
  WAIT_ECHO_RISE,
  WAIT_ECHO_FALL
};

struct EdgeReadings {
  bool frontLeft = false;
  bool frontCenter = false;
  bool frontRight = false;
  bool rearLeft = false;
  bool rearRight = false;
  bool anyFront = false;
  bool anyRear = false;
  bool any = false;
};

struct DistanceReading {
  uint16_t distanceCm = 0;
  uint32_t updatedAtMs = 0;
  uint32_t lastAttemptAtMs = 0;
  bool hasValidReading = false;
  bool lastAttemptValid = false;
};

struct OpponentReadings {
  bool leftDetected = false;
  bool rightDetected = false;
  bool any = false;
};

struct MotorRuntime {
  int8_t activePhysicalDirection = 0;
  int8_t lastPhysicalDirection = 0;
  uint32_t zeroStartedUs = 0;
  bool zeroTimingActive = false;
};

struct UltrasonicRuntime {
  UltrasonicSide activeSide = UltrasonicSide::LEFT;
  UltrasonicPhase phase = UltrasonicPhase::IDLE;
  uint32_t phaseStartedUs = 0;
  uint32_t measurementStartedUs = 0;
  uint32_t echoRiseUs = 0;
  uint32_t lastMeasurementFinishedUs = 0;
};

// ============================================================================
// 4. VARIÁVEIS GLOBAIS
// ============================================================================

RobotState robotState = RobotState::STARTUP_DELAY;
uint32_t stateStartedMs = 0;
uint32_t startupDelayMs = 0;
uint32_t lastDebugMs = 0;

EdgeReadings currentEdges;
DistanceReading leftDistance;
DistanceReading rightDistance;
OpponentReadings currentOpponent;

MotorRuntime leftMotorRuntime;
MotorRuntime rightMotorRuntime;
UltrasonicRuntime ultrasonicRuntime;

RecoveryPhase recoveryPhase = RecoveryPhase::STOPPING;
EdgeCategory recoveryCategory = EdgeCategory::NONE;
uint32_t recoveryPhaseStartedMs = 0;
uint8_t recoveryAttempt = 0;
int8_t recoveryTurnDirection = 1;  // +1: direita; -1: esquerda.
bool fallbackTurnRight = true;

bool pwmAttached = false;
bool configurationValid = false;

// ============================================================================
// 5. UTILITÁRIOS DE TEMPO E DEPURAÇÃO
// ============================================================================

bool elapsedMillis(uint32_t startedAt, uint32_t duration) {
  return static_cast<uint32_t>(millis() - startedAt) >= duration;
}

bool elapsedMicros(uint32_t startedAt, uint32_t duration) {
  return static_cast<uint32_t>(micros() - startedAt) >= duration;
}

const char* robotStateName(RobotState state) {
  switch (state) {
    case RobotState::STARTUP_DELAY: return "STARTUP_DELAY";
    case RobotState::INITIAL_ADVANCE: return "INITIAL_ADVANCE";
    case RobotState::EDGE_RECOVERY: return "EDGE_RECOVERY";
    case RobotState::SEARCH_TURN_RIGHT: return "SEARCH_TURN_RIGHT";
    case RobotState::SEARCH_TURN_LEFT: return "SEARCH_TURN_LEFT";
    case RobotState::SEARCH_FACE_BACK: return "SEARCH_FACE_BACK";
    case RobotState::ATTACK: return "ATTACK";
    case RobotState::TARGET_LOST: return "TARGET_LOST";
    case RobotState::SAFE_STOP: return "SAFE_STOP";
    default: return "UNKNOWN";
  }
}

const char* edgeCategoryName(EdgeCategory category) {
  switch (category) {
    case EdgeCategory::NONE: return "NONE";
    case EdgeCategory::FRONT: return "FRONT";
    case EdgeCategory::REAR: return "REAR";
    case EdgeCategory::BOTH: return "BOTH";
    default: return "UNKNOWN";
  }
}

void changeRobotState(RobotState newState) {
  if (robotState == newState) {
    return;
  }

  robotState = newState;
  stateStartedMs = millis();

  if (DEBUG) {
    Serial.print(F("[ESTADO] "));
    Serial.println(robotStateName(robotState));
  }
}

// ============================================================================
// 6. VALIDAÇÃO DOS PINOS
// ============================================================================

bool isKnownEsp32Gpio(uint8_t pin) {
  switch (pin) {
    case 0: case 1: case 2: case 3: case 4: case 5:
    case 12: case 13: case 14: case 15: case 16: case 17:
    case 18: case 19: case 21: case 22: case 23:
    case 25: case 26: case 27:
    case 32: case 33: case 34: case 35: case 36: case 39:
      return true;
    default:
      return false;
  }
}

bool isOutputCapableGpio(uint8_t pin) {
  return isKnownEsp32Gpio(pin) && pin < 34;
}

bool validatePinConfiguration() {
  constexpr uint8_t allPins[] = {
    PIN_MOTOR_LEFT_RPWM,
    PIN_MOTOR_LEFT_LPWM,
    PIN_MOTOR_RIGHT_RPWM,
    PIN_MOTOR_RIGHT_LPWM,
    PIN_HCSR04_LEFT_TRIG,
    PIN_HCSR04_LEFT_ECHO,
    PIN_HCSR04_RIGHT_TRIG,
    PIN_HCSR04_RIGHT_ECHO,
    PIN_EDGE_REAR_LEFT,
    PIN_EDGE_REAR_RIGHT,
    PIN_EDGE_FRONT_LEFT,
    PIN_EDGE_FRONT_CENTER,
    PIN_EDGE_FRONT_RIGHT
  };

  constexpr uint8_t outputPins[] = {
    PIN_MOTOR_LEFT_RPWM,
    PIN_MOTOR_LEFT_LPWM,
    PIN_MOTOR_RIGHT_RPWM,
    PIN_MOTOR_RIGHT_LPWM,
    PIN_HCSR04_LEFT_TRIG,
    PIN_HCSR04_RIGHT_TRIG
  };

  for (uint8_t pin : allPins) {
    if (!isKnownEsp32Gpio(pin)) {
      if (DEBUG) {
        Serial.print(F("[ERRO] GPIO invalido ou indisponivel: "));
        Serial.println(pin);
      }
      return false;
    }
  }

  for (uint8_t pin : outputPins) {
    if (!isOutputCapableGpio(pin)) {
      if (DEBUG) {
        Serial.print(F("[ERRO] GPIO nao suporta saida: "));
        Serial.println(pin);
      }
      return false;
    }
  }

  constexpr size_t pinCount = sizeof(allPins) / sizeof(allPins[0]);
  for (size_t i = 0; i < pinCount; ++i) {
    for (size_t j = i + 1; j < pinCount; ++j) {
      if (allPins[i] == allPins[j]) {
        if (DEBUG) {
          Serial.print(F("[ERRO] GPIO repetido: "));
          Serial.println(allPins[i]);
        }
        return false;
      }
    }
  }

  return true;
}

// ============================================================================
// 7. CONTROLE DOS MOTORES
// ============================================================================

int clampMotorSpeed(int speed) {
  if (speed > 255) return 255;
  if (speed < -255) return -255;
  return speed;
}

uint8_t logicalSpeedToPhysicalDuty(int speed) {
  const int limitedSpeed = abs(clampMotorSpeed(speed));
  return static_cast<uint8_t>((limitedSpeed * PWM_PHYSICAL_MAX + 127) / 255);
}

void writeMotorPins(uint8_t positivePwmPin,
                    uint8_t negativePwmPin,
                    int8_t physicalDirection,
                    uint8_t duty) {
  // Nunca deixa os dois lados do mesmo motor ativos simultaneamente.
  if (physicalDirection > 0 && duty > 0) {
    ledcWrite(negativePwmPin, 0);
    ledcWrite(positivePwmPin, duty);
  } else if (physicalDirection < 0 && duty > 0) {
    ledcWrite(positivePwmPin, 0);
    ledcWrite(negativePwmPin, duty);
  } else {
    ledcWrite(positivePwmPin, 0);
    ledcWrite(negativePwmPin, 0);
  }
}

void forceMotorOutputsOff() {
  if (pwmAttached) {
    ledcWrite(PIN_MOTOR_LEFT_RPWM, 0);
    ledcWrite(PIN_MOTOR_LEFT_LPWM, 0);
    ledcWrite(PIN_MOTOR_RIGHT_RPWM, 0);
    ledcWrite(PIN_MOTOR_RIGHT_LPWM, 0);
  } else {
    digitalWrite(PIN_MOTOR_LEFT_RPWM, LOW);
    digitalWrite(PIN_MOTOR_LEFT_LPWM, LOW);
    digitalWrite(PIN_MOTOR_RIGHT_RPWM, LOW);
    digitalWrite(PIN_MOTOR_RIGHT_LPWM, LOW);
  }
}

void applyMotorCommand(uint8_t positivePwmPin,
                       uint8_t negativePwmPin,
                       int logicalSpeed,
                       bool invertMotor,
                       MotorRuntime& runtime) {
  logicalSpeed = clampMotorSpeed(logicalSpeed);

  int8_t desiredDirection = 0;
  if (logicalSpeed > 0) desiredDirection = 1;
  if (logicalSpeed < 0) desiredDirection = -1;
  if (invertMotor) desiredDirection = -desiredDirection;

  const uint8_t desiredDuty = logicalSpeedToPhysicalDuty(logicalSpeed);
  const uint32_t nowUs = micros();

  if (desiredDirection == 0 || desiredDuty == 0) {
    writeMotorPins(positivePwmPin, negativePwmPin, 0, 0);

    if (runtime.activePhysicalDirection != 0) {
      runtime.lastPhysicalDirection = runtime.activePhysicalDirection;
      runtime.zeroStartedUs = nowUs;
      runtime.zeroTimingActive = true;
    }

    runtime.activePhysicalDirection = 0;
    return;
  }

  // Inversão solicitada enquanto o motor ainda está energizado.
  if (runtime.activePhysicalDirection != 0 &&
      runtime.activePhysicalDirection != desiredDirection) {
    writeMotorPins(positivePwmPin, negativePwmPin, 0, 0);
    runtime.lastPhysicalDirection = runtime.activePhysicalDirection;
    runtime.activePhysicalDirection = 0;
    runtime.zeroStartedUs = nowUs;
    runtime.zeroTimingActive = true;
    return;
  }

  // Aguarda o tempo morto apenas quando o novo sentido é oposto ao anterior.
  if (runtime.activePhysicalDirection == 0 &&
      runtime.zeroTimingActive &&
      runtime.lastPhysicalDirection == -desiredDirection &&
      static_cast<uint32_t>(nowUs - runtime.zeroStartedUs) < MOTOR_REVERSAL_DEADTIME_US) {
    writeMotorPins(positivePwmPin, negativePwmPin, 0, 0);
    return;
  }

  runtime.zeroTimingActive = false;
  runtime.activePhysicalDirection = desiredDirection;
  runtime.lastPhysicalDirection = desiredDirection;
  writeMotorPins(positivePwmPin, negativePwmPin, desiredDirection, desiredDuty);
}

void setMotors(int leftSpeed, int rightSpeed) {
  if (!configurationValid || !pwmAttached) {
    forceMotorOutputsOff();
    return;
  }

  applyMotorCommand(
    PIN_MOTOR_LEFT_RPWM,
    PIN_MOTOR_LEFT_LPWM,
    leftSpeed,
    INVERT_LEFT_MOTOR,
    leftMotorRuntime
  );

  applyMotorCommand(
    PIN_MOTOR_RIGHT_RPWM,
    PIN_MOTOR_RIGHT_LPWM,
    rightSpeed,
    INVERT_RIGHT_MOTOR,
    rightMotorRuntime
  );
}

void stopMotors() {
  setMotors(0, 0);
}

void moveForward(int speed) {
  speed = abs(clampMotorSpeed(speed));
  setMotors(speed, speed);
}

void moveBackward(int speed) {
  speed = abs(clampMotorSpeed(speed));
  setMotors(-speed, -speed);
}

void turnRight(int speed) {
  speed = abs(clampMotorSpeed(speed));
  setMotors(speed, -speed);
}

void turnLeft(int speed) {
  speed = abs(clampMotorSpeed(speed));
  setMotors(-speed, speed);
}

// ============================================================================
// 8. LEITURA DOS SENSORES DE BORDA
// ============================================================================

bool edgeDetectedOnPin(uint8_t pin, int detectedLevel) {
  return digitalRead(pin) == detectedLevel;
}

EdgeReadings readEdgeSensors() {
  EdgeReadings readings;

  readings.frontLeft = edgeDetectedOnPin(
    PIN_EDGE_FRONT_LEFT,
    EDGE_FRONT_DETECTED_LEVEL
  );
  readings.frontCenter = edgeDetectedOnPin(
    PIN_EDGE_FRONT_CENTER,
    EDGE_FRONT_DETECTED_LEVEL
  );
  readings.frontRight = edgeDetectedOnPin(
    PIN_EDGE_FRONT_RIGHT,
    EDGE_FRONT_DETECTED_LEVEL
  );
  readings.rearLeft = edgeDetectedOnPin(
    PIN_EDGE_REAR_LEFT,
    EDGE_REAR_DETECTED_LEVEL
  );
  readings.rearRight = edgeDetectedOnPin(
    PIN_EDGE_REAR_RIGHT,
    EDGE_REAR_DETECTED_LEVEL
  );

  readings.anyFront = readings.frontLeft ||
                      readings.frontCenter ||
                      readings.frontRight;

  readings.anyRear = readings.rearLeft || readings.rearRight;
  readings.any = readings.anyFront || readings.anyRear;

  return readings;
}

EdgeCategory classifyEdges(const EdgeReadings& readings) {
  if (readings.anyFront && readings.anyRear) return EdgeCategory::BOTH;
  if (readings.anyFront) return EdgeCategory::FRONT;
  if (readings.anyRear) return EdgeCategory::REAR;
  return EdgeCategory::NONE;
}

int8_t chooseRecoveryTurnDirection(const EdgeReadings& readings) {
  uint8_t leftRisk = 0;
  uint8_t rightRisk = 0;

  leftRisk += readings.frontLeft ? 1 : 0;
  leftRisk += readings.rearLeft ? 1 : 0;

  rightRisk += readings.frontRight ? 1 : 0;
  rightRisk += readings.rearRight ? 1 : 0;

  // Borda mais forte à esquerda: gira para a direita, e vice-versa.
  if (leftRisk > rightRisk) return 1;
  if (rightRisk > leftRisk) return -1;

  const int8_t direction = fallbackTurnRight ? 1 : -1;
  fallbackTurnRight = !fallbackTurnRight;
  return direction;
}

// ============================================================================
// 9. LEITURA NÃO BLOQUEANTE DOS HC-SR04
// ============================================================================

uint8_t activeTrigPin() {
  return ultrasonicRuntime.activeSide == UltrasonicSide::LEFT
           ? PIN_HCSR04_LEFT_TRIG
           : PIN_HCSR04_RIGHT_TRIG;
}

uint8_t activeEchoPin() {
  return ultrasonicRuntime.activeSide == UltrasonicSide::LEFT
           ? PIN_HCSR04_LEFT_ECHO
           : PIN_HCSR04_RIGHT_ECHO;
}

DistanceReading& activeDistanceReading() {
  return ultrasonicRuntime.activeSide == UltrasonicSide::LEFT
           ? leftDistance
           : rightDistance;
}

void finishUltrasonicMeasurement(bool valid, uint32_t pulseWidthUs = 0) {
  DistanceReading& reading = activeDistanceReading();
  reading.lastAttemptAtMs = millis();
  reading.lastAttemptValid = false;

  if (valid) {
    // Aproximação comum: distância em cm = tempo do eco em us / 58.
    const uint16_t distanceCm = static_cast<uint16_t>(pulseWidthUs / 58U);

    if (distanceCm >= ULTRASONIC_MIN_VALID_CM &&
        distanceCm <= ULTRASONIC_MAX_VALID_CM) {
      reading.distanceCm = distanceCm;
      reading.updatedAtMs = millis();
      reading.hasValidReading = true;
      reading.lastAttemptValid = true;
    }
  }

  digitalWrite(activeTrigPin(), LOW);
  ultrasonicRuntime.phase = UltrasonicPhase::IDLE;
  ultrasonicRuntime.lastMeasurementFinishedUs = micros();
  ultrasonicRuntime.activeSide =
    ultrasonicRuntime.activeSide == UltrasonicSide::LEFT
      ? UltrasonicSide::RIGHT
      : UltrasonicSide::LEFT;
}

void updateUltrasonicSensors() {
  const uint32_t nowUs = micros();

  switch (ultrasonicRuntime.phase) {
    case UltrasonicPhase::IDLE:
      if (static_cast<uint32_t>(nowUs - ultrasonicRuntime.lastMeasurementFinishedUs) >=
          ULTRASONIC_INTER_SENSOR_US) {
        digitalWrite(activeTrigPin(), LOW);
        ultrasonicRuntime.phase = UltrasonicPhase::TRIGGER_LOW;
        ultrasonicRuntime.phaseStartedUs = nowUs;
      }
      break;

    case UltrasonicPhase::TRIGGER_LOW:
      if (static_cast<uint32_t>(nowUs - ultrasonicRuntime.phaseStartedUs) >=
          ULTRASONIC_TRIGGER_LOW_US) {
        digitalWrite(activeTrigPin(), HIGH);
        ultrasonicRuntime.phase = UltrasonicPhase::TRIGGER_HIGH;
        ultrasonicRuntime.phaseStartedUs = nowUs;
      }
      break;

    case UltrasonicPhase::TRIGGER_HIGH:
      if (static_cast<uint32_t>(nowUs - ultrasonicRuntime.phaseStartedUs) >=
          ULTRASONIC_TRIGGER_HIGH_US) {
        digitalWrite(activeTrigPin(), LOW);
        ultrasonicRuntime.phase = UltrasonicPhase::WAIT_ECHO_RISE;
        ultrasonicRuntime.measurementStartedUs = nowUs;
      }
      break;

    case UltrasonicPhase::WAIT_ECHO_RISE:
      if (digitalRead(activeEchoPin()) == HIGH) {
        ultrasonicRuntime.echoRiseUs = nowUs;
        ultrasonicRuntime.phase = UltrasonicPhase::WAIT_ECHO_FALL;
      } else if (static_cast<uint32_t>(nowUs - ultrasonicRuntime.measurementStartedUs) >=
                 ULTRASONIC_ECHO_TIMEOUT_US) {
        finishUltrasonicMeasurement(false);
      }
      break;

    case UltrasonicPhase::WAIT_ECHO_FALL:
      if (digitalRead(activeEchoPin()) == LOW) {
        const uint32_t pulseWidthUs =
          static_cast<uint32_t>(nowUs - ultrasonicRuntime.echoRiseUs);
        finishUltrasonicMeasurement(true, pulseWidthUs);
      } else if (static_cast<uint32_t>(nowUs - ultrasonicRuntime.measurementStartedUs) >=
                 ULTRASONIC_ECHO_TIMEOUT_US) {
        finishUltrasonicMeasurement(false);
      }
      break;
  }
}

bool isFreshOpponentReading(const DistanceReading& reading) {
  if (!reading.hasValidReading) return false;

  const uint32_t ageMs = static_cast<uint32_t>(millis() - reading.updatedAtMs);
  return ageMs <= ULTRASONIC_READING_MAX_AGE_MS &&
         reading.distanceCm <= OPPONENT_DISTANCE_CM;
}

OpponentReadings readOpponentSensors() {
  OpponentReadings readings;
  readings.leftDetected = isFreshOpponentReading(leftDistance);
  readings.rightDetected = isFreshOpponentReading(rightDistance);
  readings.any = readings.leftDetected || readings.rightDetected;
  return readings;
}

// ============================================================================
// 10. RECUPERAÇÃO DE BORDA
// ============================================================================

void enterSafeStop(const __FlashStringHelper* reason) {
  const bool wasAlreadyStopped = robotState == RobotState::SAFE_STOP;
  stopMotors();
  changeRobotState(RobotState::SAFE_STOP);

  if (DEBUG && !wasAlreadyStopped) {
    Serial.print(F("[SEGURANCA] "));
    Serial.println(reason);
  }
}

void startRecoveryAttempt(const EdgeReadings& edges, bool resetAttempts) {
  if (resetAttempts) {
    recoveryAttempt = 1;
  } else {
    ++recoveryAttempt;
  }

  if (recoveryAttempt > EDGE_RECOVERY_MAX_ATTEMPTS) {
    enterSafeStop(F("Nao foi possivel sair da borda dentro do limite de tentativas."));
    return;
  }

  recoveryCategory = classifyEdges(edges);
  if (recoveryCategory == EdgeCategory::NONE) {
    changeRobotState(RobotState::SEARCH_TURN_RIGHT);
    return;
  }

  recoveryTurnDirection = chooseRecoveryTurnDirection(edges);
  recoveryPhase = RecoveryPhase::STOPPING;
  recoveryPhaseStartedMs = millis();
  stopMotors();

  if (robotState != RobotState::EDGE_RECOVERY) {
    changeRobotState(RobotState::EDGE_RECOVERY);
  }

  if (DEBUG) {
    Serial.print(F("[BORDA] Tipo="));
    Serial.print(edgeCategoryName(recoveryCategory));
    Serial.print(F(" tentativa="));
    Serial.print(recoveryAttempt);
    Serial.print(F(" giro="));
    Serial.println(recoveryTurnDirection > 0 ? F("direita") : F("esquerda"));
  }
}

void beginEdgeRecovery(const EdgeReadings& edges) {
  startRecoveryAttempt(edges, true);
}

void retryEdgeRecovery(const EdgeReadings& edges) {
  stopMotors();
  startRecoveryAttempt(edges, false);
}

void commandRecoveryTurn() {
  if (recoveryTurnDirection > 0) {
    turnRight(TURN_SPEED);
  } else {
    turnLeft(TURN_SPEED);
  }
}

void handleEdgeRecovery(const EdgeReadings& edges) {
  const EdgeCategory currentCategory = classifyEdges(edges);

  switch (recoveryPhase) {
    case RecoveryPhase::STOPPING:
      stopMotors();

      if (elapsedMillis(recoveryPhaseStartedMs, EDGE_IMMEDIATE_STOP_MS)) {
        recoveryPhaseStartedMs = millis();
        recoveryPhase = recoveryCategory == EdgeCategory::BOTH
                          ? RecoveryPhase::TURNING
                          : RecoveryPhase::ESCAPING;
      }
      break;

    case RecoveryPhase::ESCAPING:
      if (recoveryCategory == EdgeCategory::FRONT) {
        // Se a traseira alcançar a borda durante a ré, interrompe imediatamente.
        if (edges.anyRear) {
          retryEdgeRecovery(edges);
          return;
        }
        moveBackward(EDGE_ESCAPE_SPEED);
      } else if (recoveryCategory == EdgeCategory::REAR) {
        // Se a frente alcançar a borda durante o avanço, interrompe imediatamente.
        if (edges.anyFront) {
          retryEdgeRecovery(edges);
          return;
        }
        moveForward(EDGE_ESCAPE_SPEED);
      } else {
        retryEdgeRecovery(edges);
        return;
      }

      if (elapsedMillis(recoveryPhaseStartedMs, EDGE_ESCAPE_MS)) {
        stopMotors();
        recoveryPhase = RecoveryPhase::SETTLING;
        recoveryPhaseStartedMs = millis();
      }
      break;

    case RecoveryPhase::SETTLING:
      stopMotors();

      if (elapsedMillis(recoveryPhaseStartedMs, EDGE_SETTLE_MS)) {
        // Só começa a curva normal se o deslocamento linear já limpou a borda.
        if (edges.any) {
          retryEdgeRecovery(edges);
          return;
        }

        recoveryPhase = RecoveryPhase::TURNING;
        recoveryPhaseStartedMs = millis();
      }
      break;

    case RecoveryPhase::TURNING: {
      const uint32_t turnDuration = recoveryCategory == EdgeCategory::BOTH
                                      ? EDGE_BOTH_TURN_MS
                                      : EDGE_RECOVERY_TURN_MS;

      if (recoveryCategory == EdgeCategory::BOTH) {
        // Estratégia limitada: apenas gira; nunca avança nem recua neste caso.
        if (currentCategory != EdgeCategory::BOTH &&
            currentCategory != EdgeCategory::NONE) {
          retryEdgeRecovery(edges);
          return;
        }
      } else if (edges.any) {
        // Uma borda nova durante a curva tem prioridade imediata.
        retryEdgeRecovery(edges);
        return;
      }

      commandRecoveryTurn();

      if (elapsedMillis(recoveryPhaseStartedMs, turnDuration)) {
        stopMotors();
        recoveryPhase = RecoveryPhase::VERIFYING;
        recoveryPhaseStartedMs = millis();
      }
      break;
    }

    case RecoveryPhase::VERIFYING:
      stopMotors();

      if (elapsedMillis(recoveryPhaseStartedMs, EDGE_VERIFY_MS)) {
        if (edges.any) {
          retryEdgeRecovery(edges);
        } else {
          recoveryAttempt = 0;
          recoveryCategory = EdgeCategory::NONE;
          changeRobotState(RobotState::SEARCH_TURN_RIGHT);
        }
      }
      break;
  }
}

// ============================================================================
// 11. BUSCA, ATAQUE E MÁQUINA DE ESTADOS
// ============================================================================

void startAttack() {
  changeRobotState(RobotState::ATTACK);
}

void handleSearchTurnRight() {
  if (currentOpponent.any) {
    startAttack();
    return;
  }

  turnRight(SEARCH_SPEED);

  if (elapsedMillis(stateStartedMs, TURN_90_MS)) {
    changeRobotState(RobotState::SEARCH_TURN_LEFT);
  }
}

void handleSearchTurnLeft() {
  if (currentOpponent.any) {
    startAttack();
    return;
  }

  turnLeft(SEARCH_SPEED);

  if (elapsedMillis(stateStartedMs, TURN_180_MS)) {
    changeRobotState(RobotState::SEARCH_FACE_BACK);
  }
}

void handleSearchFaceBack() {
  if (currentOpponent.any) {
    startAttack();
    return;
  }

  turnLeft(SEARCH_SPEED);

  if (elapsedMillis(stateStartedMs, TURN_90_MS)) {
    // Agora está aproximadamente voltado para trás da orientação inicial.
    changeRobotState(RobotState::SEARCH_TURN_RIGHT);
  }
}

void handleAttack() {
  if (!currentOpponent.any) {
    stopMotors();
    changeRobotState(RobotState::TARGET_LOST);
    return;
  }

  if (currentOpponent.leftDetected && currentOpponent.rightDetected) {
    moveForward(ATTACK_SPEED);
  } else if (currentOpponent.leftDetected) {
    // Curva suave para a esquerda: roda esquerda mais lenta.
    setMotors(ATTACK_TURN_SPEED, ATTACK_SPEED);
  } else {
    // Curva suave para a direita: roda direita mais lenta.
    setMotors(ATTACK_SPEED, ATTACK_TURN_SPEED);
  }
}

void handleTargetLost() {
  stopMotors();

  if (currentOpponent.any) {
    startAttack();
    return;
  }

  if (elapsedMillis(stateStartedMs, TARGET_LOST_MS)) {
    changeRobotState(RobotState::SEARCH_TURN_RIGHT);
  }
}

void updateRobotState() {
  currentEdges = readEdgeSensors();
  currentOpponent = readOpponentSensors();

  if (!configurationValid || !pwmAttached) {
    enterSafeStop(F("Configuracao de pinos ou PWM invalida."));
    return;
  }

  if (robotState == RobotState::SAFE_STOP) {
    stopMotors();
    return;
  }

  // Durante o atraso inicial, a resposta segura é permanecer imóvel.
  if (robotState == RobotState::STARTUP_DELAY) {
    stopMotors();

    if (elapsedMillis(stateStartedMs, startupDelayMs)) {
      if (currentEdges.any) {
        beginEdgeRecovery(currentEdges);
      } else {
        changeRobotState(RobotState::INITIAL_ADVANCE);
      }
    }
    return;
  }

  // Borda tem prioridade sobre busca, ataque e qualquer outro movimento.
  if (robotState != RobotState::EDGE_RECOVERY && currentEdges.any) {
    beginEdgeRecovery(currentEdges);
    return;
  }

  if (robotState == RobotState::EDGE_RECOVERY) {
    handleEdgeRecovery(currentEdges);
    return;
  }

  switch (robotState) {
    case RobotState::INITIAL_ADVANCE:
      moveForward(INITIAL_ADVANCE_SPEED);
      break;

    case RobotState::SEARCH_TURN_RIGHT:
      handleSearchTurnRight();
      break;

    case RobotState::SEARCH_TURN_LEFT:
      handleSearchTurnLeft();
      break;

    case RobotState::SEARCH_FACE_BACK:
      handleSearchFaceBack();
      break;

    case RobotState::ATTACK:
      handleAttack();
      break;

    case RobotState::TARGET_LOST:
      handleTargetLost();
      break;

    case RobotState::STARTUP_DELAY:
    case RobotState::EDGE_RECOVERY:
    case RobotState::SAFE_STOP:
      break;
  }
}

// ============================================================================
// 12. MONITOR SERIAL
// ============================================================================

void printDistance(const DistanceReading& reading) {
  if (!reading.hasValidReading ||
      static_cast<uint32_t>(millis() - reading.updatedAtMs) >
        ULTRASONIC_READING_MAX_AGE_MS) {
    Serial.print(F("--"));
    return;
  }

  Serial.print(reading.distanceCm);
}

void printDebugStatus() {
  if (!DEBUG || !elapsedMillis(lastDebugMs, DEBUG_INTERVAL_MS)) {
    return;
  }

  lastDebugMs = millis();

  Serial.print(F("[STATUS] estado="));
  Serial.print(robotStateName(robotState));

  Serial.print(F(" | frente="));
  Serial.print(currentEdges.frontLeft ? '1' : '0');
  Serial.print(currentEdges.frontCenter ? '1' : '0');
  Serial.print(currentEdges.frontRight ? '1' : '0');

  Serial.print(F(" | traseira="));
  Serial.print(currentEdges.rearLeft ? '1' : '0');
  Serial.print(currentEdges.rearRight ? '1' : '0');

  Serial.print(F(" | HC-L="));
  printDistance(leftDistance);
  Serial.print(F("cm HC-R="));
  printDistance(rightDistance);
  Serial.println(F("cm"));
}

// ============================================================================
// 13. INICIALIZAÇÃO
// ============================================================================

void configurePins() {
  pinMode(PIN_MOTOR_LEFT_RPWM, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_LPWM, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_RPWM, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_LPWM, OUTPUT);

  // Garante nível baixo antes de ativar o periférico PWM.
  digitalWrite(PIN_MOTOR_LEFT_RPWM, LOW);
  digitalWrite(PIN_MOTOR_LEFT_LPWM, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_RPWM, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_LPWM, LOW);

  pinMode(PIN_HCSR04_LEFT_TRIG, OUTPUT);
  pinMode(PIN_HCSR04_RIGHT_TRIG, OUTPUT);
  digitalWrite(PIN_HCSR04_LEFT_TRIG, LOW);
  digitalWrite(PIN_HCSR04_RIGHT_TRIG, LOW);

  pinMode(PIN_HCSR04_LEFT_ECHO, INPUT);
  pinMode(PIN_HCSR04_RIGHT_ECHO, INPUT);

  // GPIO34 e GPIO35 são somente entrada e não possuem pull-up/pull-down interno.
  pinMode(PIN_EDGE_REAR_LEFT, INPUT);
  pinMode(PIN_EDGE_REAR_RIGHT, INPUT);
  pinMode(PIN_EDGE_FRONT_LEFT, INPUT);
  pinMode(PIN_EDGE_FRONT_CENTER, INPUT);
  pinMode(PIN_EDGE_FRONT_RIGHT, INPUT);
}

bool configurePwm() {
  const bool leftRpwmOk = ledcAttach(
    PIN_MOTOR_LEFT_RPWM,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  const bool leftLpwmOk = ledcAttach(
    PIN_MOTOR_LEFT_LPWM,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  const bool rightRpwmOk = ledcAttach(
    PIN_MOTOR_RIGHT_RPWM,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  const bool rightLpwmOk = ledcAttach(
    PIN_MOTOR_RIGHT_LPWM,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  return leftRpwmOk && leftLpwmOk && rightRpwmOk && rightLpwmOk;
}

void setup() {
  if (DEBUG) {
    Serial.begin(SERIAL_BAUD_RATE);
  }

  configurePins();
  forceMotorOutputsOff();

  configurationValid = validatePinConfiguration();

  if (configurationValid) {
    pwmAttached = configurePwm();
  }

  forceMotorOutputsOff();

  if (!configurationValid || !pwmAttached) {
    robotState = RobotState::SAFE_STOP;
    stateStartedMs = millis();

    if (DEBUG) {
      Serial.println(F("[ERRO] Inicializacao abortada. Motores permanecerao desligados."));
    }
    return;
  }

  startupDelayMs = esp_random() % (STARTUP_DELAY_MAX_MS + 1U);
  robotState = RobotState::STARTUP_DELAY;
  stateStartedMs = millis();
  ultrasonicRuntime.lastMeasurementFinishedUs = micros();

  if (DEBUG) {
    Serial.println(F("[OK] Carrinho de sumo inicializado."));
    Serial.print(F("[STARTUP] Atraso aleatorio: "));
    Serial.print(startupDelayMs);
    Serial.println(F(" ms"));
    Serial.print(F("[PWM] Limite fisico: "));
    Serial.print(PWM_PHYSICAL_MAX);
    Serial.println(F("/255"));
  }
}

void loop() {
  // Ordem deliberada: borda é lida na máquina de estados em toda iteração.
  updateUltrasonicSensors();
  updateRobotState();
  printDebugStatus();
}
