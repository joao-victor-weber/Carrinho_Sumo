#include <Arduino.h>

// Mapeamento dos drivers HW-039/BTS7960 conforme o README.md.
// R_EN e L_EN dos dois drivers devem permanecer ligados a 3,3 V.
constexpr uint8_t PIN_MOTOR_LEFT_RPWM = 13;
constexpr uint8_t PIN_MOTOR_LEFT_LPWM = 14;
constexpr uint8_t PIN_MOTOR_RIGHT_RPWM = 19;
constexpr uint8_t PIN_MOTOR_RIGHT_LPWM = 18;

constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr uint8_t PWM_PHYSICAL_MAX = 180;

// Valor moderado para o primeiro teste. Pode ser aumentado até
// PWM_PHYSICAL_MAX se necessário.
constexpr uint8_t TEST_PWM = 120;
static_assert(TEST_PWM <= PWM_PHYSICAL_MAX, "PWM de teste acima do limite físico");

constexpr uint32_t MOVEMENT_TIME_MS = 2000;
constexpr uint32_t STARTUP_WAIT_MS = 3000;
constexpr uint32_t DIRECTION_CHANGE_PAUSE_MS = 500;
constexpr uint32_t CYCLE_PAUSE_MS = 1000;

void stopMotors() {
  ledcWrite(PIN_MOTOR_LEFT_RPWM, 0);
  ledcWrite(PIN_MOTOR_LEFT_LPWM, 0);
  ledcWrite(PIN_MOTOR_RIGHT_RPWM, 0);
  ledcWrite(PIN_MOTOR_RIGHT_LPWM, 0);
}

void moveForward() {
  // O motor direito está invertido fisicamente na montagem atual.
  // Esquerdo para frente: RPWM; direito para frente: LPWM.
  ledcWrite(PIN_MOTOR_LEFT_LPWM, 0);
  ledcWrite(PIN_MOTOR_RIGHT_RPWM, 0);
  ledcWrite(PIN_MOTOR_LEFT_RPWM, TEST_PWM);
  ledcWrite(PIN_MOTOR_RIGHT_LPWM, TEST_PWM);
}

void moveBackward() {
  // Esquerdo para trás: LPWM; direito para trás: RPWM.
  ledcWrite(PIN_MOTOR_LEFT_RPWM, 0);
  ledcWrite(PIN_MOTOR_RIGHT_LPWM, 0);
  ledcWrite(PIN_MOTOR_LEFT_LPWM, TEST_PWM);
  ledcWrite(PIN_MOTOR_RIGHT_RPWM, TEST_PWM);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_MOTOR_LEFT_RPWM, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_LPWM, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_RPWM, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_LPWM, OUTPUT);

  // Mantém os motores desligados antes de conectar o periférico PWM.
  digitalWrite(PIN_MOTOR_LEFT_RPWM, LOW);
  digitalWrite(PIN_MOTOR_LEFT_LPWM, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_RPWM, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_LPWM, LOW);

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

  stopMotors();

  if (!leftRpwmOk || !leftLpwmOk || !rightRpwmOk || !rightLpwmOk) {
    Serial.println(F("[ERRO] Não foi possível configurar o PWM."));

    while (true) {
      stopMotors();
      delay(1000);
    }
  }

  Serial.println(F("[OK] Teste dos motores iniciado."));
  Serial.println(F("Mantenha as rodas suspensas durante o primeiro teste."));
  delay(STARTUP_WAIT_MS);
}

void loop() {
  Serial.println(F("Frente por 2 segundos"));
  moveForward();
  delay(MOVEMENT_TIME_MS);

  Serial.println(F("Parado antes de inverter"));
  stopMotors();
  delay(DIRECTION_CHANGE_PAUSE_MS);

  Serial.println(F("Trás por 2 segundos"));
  moveBackward();
  delay(MOVEMENT_TIME_MS);

  Serial.println(F("Parado"));
  stopMotors();
  delay(CYCLE_PAUSE_MS);
}
