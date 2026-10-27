#include <Arduino.h>
#include <esp_random.h>

// =====================================================
// CONFIGURAÇÃO GERAL
// =====================================================

constexpr bool DEBUG = true;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;
constexpr uint32_t DEBUG_INTERVAL_MS = 250;


// =====================================================
// PINOS DOS MOTORES - HW-039 / BTS7960
// =====================================================

// Motor esquerdo
constexpr uint8_t MOTOR_ESQ_RPWM = 27;
constexpr uint8_t MOTOR_ESQ_LPWM = 26;

// Motor direito
constexpr uint8_t MOTOR_DIR_RPWM = 25;
constexpr uint8_t MOTOR_DIR_LPWM = 33;


// =====================================================
// CONFIGURAÇÃO DOS MOTORES
// =====================================================

constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;

// PWM físico máximo testado
constexpr int PWM_MAXIMO = 180;

// Tempo morto de segurança ao inverter o sentido do motor
constexpr uint32_t MOTOR_DEADTIME_US = 3000;
// Sentido físico dos motores no chassi.
// O motor direito está montado espelhado.
constexpr bool INVERTER_MOTOR_ESQUERDO = false;
constexpr bool INVERTER_MOTOR_DIREITO = true;

// =====================================================
// VELOCIDADES
// =====================================================

// Valores usados diretamente como PWM de 0 a 180
constexpr int VELOCIDADE_INICIAL = 155;
constexpr int VELOCIDADE_BUSCA = 160;
constexpr int VELOCIDADE_GIRO = 170;
constexpr int VELOCIDADE_FUGA_BORDA = 180;

// Valores lógicos de ataque de 0 a 255.
// São convertidos para PWM físico de 0 a 180.
constexpr int VELOCIDADE_ATAQUE = 255;
constexpr int VELOCIDADE_CURVA_ATAQUE = 190;


// =====================================================
// SENSORES DE BORDA - FRONTAIS
// =====================================================

constexpr uint8_t SENSOR_BORDA_ESQUERDO = 13;
constexpr uint8_t SENSOR_BORDA_DIREITO = 32;

// Branco = borda
constexpr int NIVEL_BORDA = HIGH;


// =====================================================
// HC-SR04 ESQUERDO
// =====================================================

constexpr uint8_t HC_ESQ_TRIG = 5;
constexpr uint8_t HC_ESQ_ECHO = 18;


// =====================================================
// HC-SR04 DIREITO
// =====================================================

constexpr uint8_t HC_DIR_TRIG = 19;
constexpr uint8_t HC_DIR_ECHO = 23;


// =====================================================
// CONFIGURAÇÃO DOS ULTRASSÔNICOS
// =====================================================

constexpr float DISTANCIA_OPONENTE_CM = 70.0f;

constexpr float DISTANCIA_MINIMA_CM = 2.0f;
constexpr float DISTANCIA_MAXIMA_CM = 200.0f;

constexpr uint32_t ULTRASONIC_TIMEOUT_US = 15000;

// Intervalo entre os disparos dos sensores
constexpr uint32_t ULTRASONIC_INTERVAL_US = 30000;

// Tempo máximo que uma leitura continua válida
constexpr uint32_t ULTRASONIC_READING_MAX_AGE_MS = 180;


// =====================================================
// CONFIGURAÇÃO DA TRIANGULAÇÃO
// =====================================================

// Distância física entre os centros dos HC-SR04
constexpr float DISTANCIA_ENTRE_HC_CM = 6.86f;

// Velocidade aproximada do som em cm/us
constexpr float VELOCIDADE_SOM_CM_US = 0.0343f;

// Pulso de disparo
constexpr uint32_t HC_TRIGGER_LOW_US = 3;
constexpr uint32_t HC_TRIGGER_HIGH_US = 10;


// =====================================================
// CONFIGURAÇÃO DO ATAQUE
// =====================================================

// Até este ângulo, considera o adversário centralizado
constexpr float ANGULO_CENTRAL_GRAUS = 6.0f;

// A partir deste ângulo usa correção máxima
constexpr float ANGULO_CORRECAO_MAX_GRAUS = 35.0f;

// Diferença aceitável quando a triangulação falhar
constexpr float DIFERENCA_DISTANCIA_CENTRO_CM = 4.0f;


// =====================================================
// TEMPOS DA COMPETIÇÃO
// =====================================================

// 0 a 3 segundos
constexpr uint32_t STARTUP_DELAY_MAX_MS = 3000;

// Tempos iniciais para calibração
constexpr uint32_t TURN_90_MS = 350;
constexpr uint32_t TURN_180_MS = 700;

// Recuperação da borda
constexpr uint32_t EDGE_ESCAPE_MS = 350;
constexpr uint32_t EDGE_STOP_MS = 50;
constexpr uint32_t EDGE_TURN_MS = 300;
// Tempo para confirmar que o robô realmente saiu da borda.
constexpr uint32_t EDGE_CLEAR_CONFIRM_MS = 80;

// Máximo de ciclos consecutivos de recuperação.
constexpr uint8_t MAX_TENTATIVAS_RECUPERACAO = 4;
// Tempo tolerado sem enxergar o adversário
constexpr uint32_t TARGET_LOST_MS = 300;


// =====================================================
// ESTADOS PRINCIPAIS DO ROBÔ
// =====================================================

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


// =====================================================
// FASES DA RECUPERAÇÃO DE BORDA
// =====================================================

enum class EdgeRecoveryState : uint8_t {
  STOP,
  BACKWARD,
  TURN,
  FINISHED
};


// =====================================================
// ESTADOS DO ULTRASSÔNICO
// =====================================================

enum class UltrasonicState : uint8_t {
  IDLE,
  TRIGGER_LOW,
  TRIGGER_HIGH,
  WAIT_ECHO_HIGH,
  WAIT_ECHO_LOW
};


// =====================================================
// SENSOR ULTRASSÔNICO ATUAL
// =====================================================

enum class UltrasonicSensor : uint8_t {
  LEFT,
  RIGHT
};


// =====================================================
// MODO DE RASTREAMENTO
// =====================================================

enum class OpponentTrackingMode : uint8_t {
  NONE,
  LEFT_ONLY,
  RIGHT_ONLY,
  BOTH,
  TRIANGULATED
};


// =====================================================
// ESTADOS ATUAIS
// =====================================================

RobotState estadoAtual =
  RobotState::STARTUP_DELAY;

EdgeRecoveryState estadoRecuperacao =
  EdgeRecoveryState::STOP;

UltrasonicState estadoUltrassom =
  UltrasonicState::IDLE;

UltrasonicSensor sensorUltrassomAtual =
  UltrasonicSensor::LEFT;

OpponentTrackingMode modoRastreamento =
  OpponentTrackingMode::NONE;


// =====================================================
// VARIÁVEIS DOS SENSORES DE BORDA
// =====================================================

bool bordaEsquerda = false;
bool bordaDireita = false;
bool bordaDetectada = false;


// =====================================================
// VARIÁVEIS DOS HC-SR04
// =====================================================

float distanciaEsquerda = -1.0f;
float distanciaDireita = -1.0f;

bool oponenteEsquerda = false;
bool oponenteDireita = false;
bool oponenteDetectado = false;


// =====================================================
// POSIÇÃO ESTIMADA DO OPONENTE
// =====================================================

// X negativo = esquerda
// X positivo = direita
float oponenteXcm = 0.0f;

// Distância para frente
float oponenteYcm = 0.0f;

// Distância ao centro do robô
float distanciaOponenteCentroCm = -1.0f;

// Negativo = esquerda
// Positivo = direita
float anguloOponenteGraus = 0.0f;

bool triangulacaoValida = false;


// =====================================================
// CONTROLE DE TEMPO
// =====================================================

uint32_t inicioEstadoMs = 0;

uint32_t inicioRecuperacaoMs = 0;

uint32_t ultimoOponenteMs = 0;

uint32_t ultimoDebugMs = 0;

uint32_t tempoStartupMs = 0;


// =====================================================
// CONTROLE DOS ULTRASSÔNICOS
// =====================================================

uint32_t ultrasonicTimestampUs = 0;

uint32_t echoStartUs = 0;

uint32_t ultimaMedicaoUltrassomUs = 0;

uint32_t ultimaLeituraEsquerdaMs = 0;
uint32_t ultimaLeituraDireitaMs = 0;


// =====================================================
// CONTROLE DA BUSCA / RECUPERAÇÃO
// =====================================================

// true = direita
// false = esquerda
bool recuperarGirandoDireita = true;

// Quantas recuperações consecutivas foram necessárias.
uint8_t tentativasRecuperacao = 0;

uint32_t ciclosBusca = 0;
// =====================================================
// VALIDAÇÃO DO HARDWARE
// =====================================================

// Só fica true se todos os GPIOs passarem na validação.
bool configuracaoHardwareValida = false;
// =====================================================
// ESTADO INTERNO DOS MOTORES
// =====================================================
//
// ultimoSentido:
//   +1 = RPWM
//   -1 = LPWM
//    0 = nunca foi acionado
//
// ativo:
//   true  = motor recebendo PWM
//   false = RPWM e LPWM estão em zero
//
// desligadoDesdeUs:
//   instante em que o motor foi desligado.
//   Usado para calcular o deadtime sem delay().
// =====================================================

struct MotorRuntime {

  int8_t ultimoSentido = 0;

  bool ativo = false;

  uint32_t desligadoDesdeUs = 0;
};


// Um estado independente para cada motor
MotorRuntime estadoMotorEsquerdo;
MotorRuntime estadoMotorDireito;

// =====================================================
// FUNÇÕES AUXILIARES DOS HC-SR04
// =====================================================

uint8_t getTrigSensorAtual() {

  if (
    sensorUltrassomAtual ==
    UltrasonicSensor::LEFT
  ) {
    return HC_ESQ_TRIG;
  }

  return HC_DIR_TRIG;
}


uint8_t getEchoSensorAtual() {

  if (
    sensorUltrassomAtual ==
    UltrasonicSensor::LEFT
  ) {
    return HC_ESQ_ECHO;
  }

  return HC_DIR_ECHO;
}


// =====================================================
// VALIDAÇÃO DA DISTÂNCIA
// =====================================================

bool distanciaUltrassonicaValida(
  float distancia
) {

  if (distancia < DISTANCIA_MINIMA_CM) {
    return false;
  }

  if (distancia > DISTANCIA_MAXIMA_CM) {
    return false;
  }

  return true;
}


// =====================================================
// VERIFICA SE A LEITURA AINDA É RECENTE
// =====================================================

bool leituraAindaValida(
  float distancia,
  uint32_t momentoLeitura
) {

  if (!distanciaUltrassonicaValida(distancia)) {
    return false;
  }

  if (momentoLeitura == 0) {
    return false;
  }

  if (
    millis() - momentoLeitura >
    ULTRASONIC_READING_MAX_AGE_MS
  ) {
    return false;
  }

  return true;
}


// =====================================================
// REGISTRA LEITURA DO HC-SR04
// =====================================================

void registrarLeituraUltrassonica(
  float distancia
) {

  uint32_t agoraMs = millis();

  bool valida =
    distanciaUltrassonicaValida(distancia);


  // SENSOR ESQUERDO
  if (
    sensorUltrassomAtual ==
    UltrasonicSensor::LEFT
  ) {

    if (valida) {

      distanciaEsquerda = distancia;
      ultimaLeituraEsquerdaMs = agoraMs;

    } else {

      distanciaEsquerda = -1.0f;
      ultimaLeituraEsquerdaMs = 0;
    }

    return;
  }


  // SENSOR DIREITO
  if (valida) {

    distanciaDireita = distancia;
    ultimaLeituraDireitaMs = agoraMs;

  } else {

    distanciaDireita = -1.0f;
    ultimaLeituraDireitaMs = 0;
  }
}


// =====================================================
// FINALIZA UMA MEDIÇÃO E TROCA O SENSOR
// =====================================================

void finalizarMedicaoUltrassonica() {

  ultimaMedicaoUltrassomUs = micros();


  if (
    sensorUltrassomAtual ==
    UltrasonicSensor::LEFT
  ) {

    sensorUltrassomAtual =
      UltrasonicSensor::RIGHT;

  } else {

    sensorUltrassomAtual =
      UltrasonicSensor::LEFT;
  }


  estadoUltrassom =
    UltrasonicState::IDLE;
}


// =====================================================
// LEITURA NÃO BLOQUEANTE DOS HC-SR04
// =====================================================

void atualizarUltrassonicos() {

  uint32_t agoraUs = micros();

  uint8_t trig =
    getTrigSensorAtual();

  uint8_t echo =
    getEchoSensorAtual();


  switch (estadoUltrassom) {


    // -------------------------------------------------
    // AGUARDA PRÓXIMA MEDIÇÃO
    // -------------------------------------------------

    case UltrasonicState::IDLE:

      if (
        (uint32_t)(
          agoraUs -
          ultimaMedicaoUltrassomUs
        ) >= ULTRASONIC_INTERVAL_US
      ) {

        digitalWrite(trig, LOW);

        ultrasonicTimestampUs =
          agoraUs;

        estadoUltrassom =
          UltrasonicState::TRIGGER_LOW;
      }

      break;


    // -------------------------------------------------
    // GARANTE TRIG EM LOW
    // -------------------------------------------------

    case UltrasonicState::TRIGGER_LOW:

      if (
        (uint32_t)(
          agoraUs -
          ultrasonicTimestampUs
        ) >= HC_TRIGGER_LOW_US
      ) {

        digitalWrite(trig, HIGH);

        ultrasonicTimestampUs =
          agoraUs;

        estadoUltrassom =
          UltrasonicState::TRIGGER_HIGH;
      }

      break;


    // -------------------------------------------------
    // PULSO DO TRIGGER
    // -------------------------------------------------

    case UltrasonicState::TRIGGER_HIGH:

      if (
        (uint32_t)(
          agoraUs -
          ultrasonicTimestampUs
        ) >= HC_TRIGGER_HIGH_US
      ) {

        digitalWrite(trig, LOW);

        ultrasonicTimestampUs =
          agoraUs;

        estadoUltrassom =
          UltrasonicState::WAIT_ECHO_HIGH;
      }

      break;


    // -------------------------------------------------
    // ESPERA INÍCIO DO ECHO
    // -------------------------------------------------

    case UltrasonicState::WAIT_ECHO_HIGH:

      if (digitalRead(echo) == HIGH) {

        echoStartUs = agoraUs;

        estadoUltrassom =
          UltrasonicState::WAIT_ECHO_LOW;
      }

      else if (
        (uint32_t)(
          agoraUs -
          ultrasonicTimestampUs
        ) >= ULTRASONIC_TIMEOUT_US
      ) {

        registrarLeituraUltrassonica(
          -1.0f
        );

        finalizarMedicaoUltrassonica();
      }

      break;


    // -------------------------------------------------
    // ESPERA FINAL DO ECHO
    // -------------------------------------------------

    case UltrasonicState::WAIT_ECHO_LOW:

      if (digitalRead(echo) == LOW) {

        uint32_t duracaoUs =
          agoraUs - echoStartUs;


        float distancia =
          (
            duracaoUs *
            VELOCIDADE_SOM_CM_US
          ) / 2.0f;


        registrarLeituraUltrassonica(
          distancia
        );

        finalizarMedicaoUltrassonica();
      }

      else if (
        (uint32_t)(
          agoraUs -
          echoStartUs
        ) >= ULTRASONIC_TIMEOUT_US
      ) {

        registrarLeituraUltrassonica(
          -1.0f
        );

        finalizarMedicaoUltrassonica();
      }

      break;
  }
}


// =====================================================
// DETECÇÃO DO OPONENTE
// =====================================================

void atualizarDeteccaoOponente() {

  bool esquerdaValida =
    leituraAindaValida(
      distanciaEsquerda,
      ultimaLeituraEsquerdaMs
    );


  bool direitaValida =
    leituraAindaValida(
      distanciaDireita,
      ultimaLeituraDireitaMs
    );


  oponenteEsquerda =
    esquerdaValida &&
    distanciaEsquerda <=
      DISTANCIA_OPONENTE_CM;


  oponenteDireita =
    direitaValida &&
    distanciaDireita <=
      DISTANCIA_OPONENTE_CM;


  oponenteDetectado =
    oponenteEsquerda ||
    oponenteDireita;


  if (oponenteDetectado) {
    ultimoOponenteMs = millis();
  }
}


// =====================================================
// TRIANGULAÇÃO DO OPONENTE
// =====================================================

bool triangularOponente() {

  triangulacaoValida = false;

  oponenteXcm = 0.0f;
  oponenteYcm = 0.0f;

  distanciaOponenteCentroCm = -1.0f;

  anguloOponenteGraus = 0.0f;


  if (DISTANCIA_ENTRE_HC_CM <= 0.0f) {
    return false;
  }


  // Os dois sensores precisam detectar
  if (
    !oponenteEsquerda ||
    !oponenteDireita
  ) {
    return false;
  }


  float dE = distanciaEsquerda;
  float dD = distanciaDireita;

  float B = DISTANCIA_ENTRE_HC_CM;


  // As circunferências precisam se cruzar
  if (fabsf(dE - dD) > B) {
    return false;
  }

  if ((dE + dD) < B) {
    return false;
  }


  // Posição lateral
  float x =
    (
      (dE * dE) -
      (dD * dD)
    )
    /
    (2.0f * B);


  float metadeB =
    B / 2.0f;


  float termoX =
    x + metadeB;


  float yQuadrado =
    (dE * dE) -
    (termoX * termoX);


  if (yQuadrado < 0.0f) {
    return false;
  }


  float y =
    sqrtf(yQuadrado);


  oponenteXcm = x;
  oponenteYcm = y;


  distanciaOponenteCentroCm =
    sqrtf(
      (x * x) +
      (y * y)
    );


  anguloOponenteGraus =
    atan2f(x, y) *
    180.0f /
    PI;


  triangulacaoValida = true;

  return true;
}


// =====================================================
// RASTREAMENTO DO OPONENTE
// =====================================================

void atualizarRastreamentoOponente() {

  atualizarDeteccaoOponente();

  triangulacaoValida = false;


  // Nenhum sensor detecta
  if (
    !oponenteEsquerda &&
    !oponenteDireita
  ) {

    modoRastreamento =
      OpponentTrackingMode::NONE;

    distanciaOponenteCentroCm =
      -1.0f;

    return;
  }


  // Somente esquerdo
  if (
    oponenteEsquerda &&
    !oponenteDireita
  ) {

    modoRastreamento =
      OpponentTrackingMode::LEFT_ONLY;

    distanciaOponenteCentroCm =
      distanciaEsquerda;

    return;
  }


  // Somente direito
  if (
    !oponenteEsquerda &&
    oponenteDireita
  ) {

    modoRastreamento =
      OpponentTrackingMode::RIGHT_ONLY;

    distanciaOponenteCentroCm =
      distanciaDireita;

    return;
  }


  // Os dois sensores detectam
  modoRastreamento =
    OpponentTrackingMode::BOTH;


  if (triangularOponente()) {

    modoRastreamento =
      OpponentTrackingMode::TRIANGULATED;
  }
}


// =====================================================
// CONVERSÃO DE VELOCIDADE DO ATAQUE
// =====================================================

// Converte a escala lógica 0-255
// para o PWM físico 0-180.
int converterVelocidadeParaPWM(
  int velocidade
) {

  velocidade =
    constrain(
      velocidade,
      0,
      255
    );


  return map(
    velocidade,
    0,
    255,
    0,
    PWM_MAXIMO
  );
}


// =====================================================
// LIMITE DO PWM NORMAL
// =====================================================

int limitarPWM(
  int velocidade
) {

  return constrain(
    velocidade,
    0,
    PWM_MAXIMO
  );
}
// =====================================================
// CONTROLE SEGURO DOS MOTORES
// =====================================================


// =====================================================
// DESLIGA FISICAMENTE UM MOTOR
// =====================================================

void desligarMotor(
  uint8_t rpwm,
  uint8_t lpwm
) {

  ledcWrite(rpwm, 0);
  ledcWrite(lpwm, 0);
}


// =====================================================
// APLICA PWM FÍSICO AO HW-039
// =====================================================
//
// sentido:
//
// +1 -> RPWM
// -1 -> LPWM
//
// Nunca ativa RPWM e LPWM simultaneamente.
// =====================================================

void aplicarMotorFisico(
  uint8_t rpwm,
  uint8_t lpwm,
  int8_t sentido,
  int pwm
) {

  pwm = constrain(
    pwm,
    0,
    PWM_MAXIMO
  );


  // -----------------------------------------
  // RPWM
  // -----------------------------------------

  if (sentido > 0) {

    // Primeiro garante LPWM desligado
    ledcWrite(lpwm, 0);

    // Depois ativa RPWM
    ledcWrite(rpwm, pwm);

    return;
  }


  // -----------------------------------------
  // LPWM
  // -----------------------------------------

  if (sentido < 0) {

    // Primeiro garante RPWM desligado
    ledcWrite(rpwm, 0);

    // Depois ativa LPWM
    ledcWrite(lpwm, pwm);

    return;
  }


  // -----------------------------------------
  // PARADO
  // -----------------------------------------

  desligarMotor(
    rpwm,
    lpwm
  );
}


// =====================================================
// CONTROLADOR NÃO BLOQUEANTE DE UM MOTOR
// =====================================================
//
// velocidadeLogica:
//
// positivo -> frente
// negativo -> ré
// zero     -> parado
//
// A função:
// - reconhece inversão;
// - desliga primeiro o motor;
// - espera MOTOR_DEADTIME_US;
// - só então permite o sentido contrário.
//
// Não usa delay().
// Não usa delayMicroseconds().
// =====================================================

void controlarMotor(
  uint8_t rpwm,
  uint8_t lpwm,
  int velocidadeLogica,
  bool invertido,
  MotorRuntime &estado
) {

  // Limite físico permitido.
  velocidadeLogica =
    constrain(
      velocidadeLogica,
      -PWM_MAXIMO,
      PWM_MAXIMO
    );


  // ===================================================
  // CORRIGE MOTOR ESPELHADO
  // ===================================================

  int velocidadeFisica =
    invertido
      ? -velocidadeLogica
      : velocidadeLogica;


  // ===================================================
  // DETERMINA SENTIDO
  // ===================================================

  int8_t sentidoDesejado = 0;


  if (velocidadeFisica > 0) {

    sentidoDesejado = 1;

  } else if (velocidadeFisica < 0) {

    sentidoDesejado = -1;
  }


  // ===================================================
  // CALCULA PWM
  // ===================================================

  int pwmDesejado;

  if (velocidadeFisica >= 0) {

    pwmDesejado =
      velocidadeFisica;

  } else {

    pwmDesejado =
      -velocidadeFisica;
  }


  uint32_t agoraUs =
    micros();


  // ===================================================
  // COMANDO = PARAR
  // ===================================================

  if (sentidoDesejado == 0) {

    // Só registra o instante da parada
    // na primeira vez.
    if (estado.ativo) {

      desligarMotor(
        rpwm,
        lpwm
      );

      estado.ativo = false;

      estado.desligadoDesdeUs =
        agoraUs;
    }

    return;
  }


  // ===================================================
  // MOTOR ESTÁ ATIVO
  // ===================================================

  if (estado.ativo) {


    // -------------------------------------------------
    // CONTINUA NO MESMO SENTIDO
    // -------------------------------------------------

    if (
      sentidoDesejado ==
      estado.ultimoSentido
    ) {

      // Apenas atualiza a velocidade.
      //
      // NÃO cria deadtime porque não houve
      // inversão.
      aplicarMotorFisico(
        rpwm,
        lpwm,
        sentidoDesejado,
        pwmDesejado
      );

      return;
    }


    // -------------------------------------------------
    // INVERSÃO DETECTADA
    // -------------------------------------------------
    //
    // Exemplo:
    //
    // frente -> ré
    //
    // Primeiro desligamos completamente.
    // -------------------------------------------------

    desligarMotor(
      rpwm,
      lpwm
    );


    estado.ativo = false;

    estado.desligadoDesdeUs =
      agoraUs;


    // IMPORTANTE:
    //
    // Não ligamos o sentido contrário
    // nesta mesma chamada.
    //
    // O próximo loop() verificará se
    // os 3000 us já passaram.
    return;
  }


  // ===================================================
  // MOTOR ESTÁ DESLIGADO
  // ===================================================


  // ---------------------------------------------------
  // VERIFICA SE ESTAMOS TENTANDO INVERTER
  // ---------------------------------------------------

  if (
    estado.ultimoSentido != 0 &&
    sentidoDesejado !=
      estado.ultimoSentido
  ) {

    uint32_t tempoDesligado =
      agoraUs -
      estado.desligadoDesdeUs;


    // Deadtime ainda não terminou.
    if (
      tempoDesligado <
      MOTOR_DEADTIME_US
    ) {

      // Garante ambos desligados.
      desligarMotor(
        rpwm,
        lpwm
      );

      return;
    }
  }


  // ===================================================
  // PODE LIGAR O MOTOR
  // ===================================================

  aplicarMotorFisico(
    rpwm,
    lpwm,
    sentidoDesejado,
    pwmDesejado
  );


  estado.ultimoSentido =
    sentidoDesejado;

  estado.ativo = true;
}


// =====================================================
// CONTROLE DOS DOIS MOTORES
// =====================================================
//
// Valores:
//   positivo = frente
//   negativo = ré
//   zero     = parado
//
// Exemplo:
//
// setMotors(150, 150);
// -> frente
//
// setMotors(-150, -150);
// -> ré
//
// setMotors(150, -150);
// -> direita
//
// setMotors(-150, 150);
// -> esquerda
// =====================================================

void setMotors(
  int velocidadeEsquerda,
  int velocidadeDireita
) {

  controlarMotor(
    MOTOR_ESQ_RPWM,
    MOTOR_ESQ_LPWM,
    velocidadeEsquerda,
    INVERTER_MOTOR_ESQUERDO,
    estadoMotorEsquerdo
  );


  controlarMotor(
    MOTOR_DIR_RPWM,
    MOTOR_DIR_LPWM,
    velocidadeDireita,
    INVERTER_MOTOR_DIREITO,
    estadoMotorDireito
  );
}


// =====================================================
// PARAR MOTORES
// =====================================================

void pararMotores() {

  setMotors(
    0,
    0
  );
}


// =====================================================
// FRENTE
// =====================================================

void frente(
  int velocidade
) {

  int pwm =
    limitarPWM(
      velocidade
    );


  setMotors(
    pwm,
    pwm
  );
}


// =====================================================
// RÉ
// =====================================================

void re(
  int velocidade
) {

  int pwm =
    limitarPWM(
      velocidade
    );


  setMotors(
    -pwm,
    -pwm
  );
}


// =====================================================
// GIRAR PARA DIREITA
// =====================================================
//
// Esquerdo -> frente
// Direito  -> ré
// =====================================================

void direita(
  int velocidade
) {

  int pwm =
    limitarPWM(
      velocidade
    );


  setMotors(
    pwm,
    -pwm
  );
}


// =====================================================
// GIRAR PARA ESQUERDA
// =====================================================
//
// Esquerdo -> ré
// Direito  -> frente
// =====================================================

void esquerda(
  int velocidade
) {

  int pwm =
    limitarPWM(
      velocidade
    );


  setMotors(
    -pwm,
    pwm
  );
}


// =====================================================
// FRENTE DIFERENCIAL
// =====================================================
//
// Utilizada durante o ataque.
//
// Ambos continuam para frente,
// mas podem usar velocidades diferentes.
// =====================================================

void frenteDiferencial(
  int velocidadeEsquerda,
  int velocidadeDireita
) {

  int pwmEsquerdo =
    converterVelocidadeParaPWM(
      velocidadeEsquerda
    );


  int pwmDireito =
    converterVelocidadeParaPWM(
      velocidadeDireita
    );


  setMotors(
    pwmEsquerdo,
    pwmDireito
  );
}
// =====================================================
// ATAQUE PROPORCIONAL AO ÂNGULO
// =====================================================

void atacarComAngulo(
  float angulo
) {

  float anguloAbsoluto =
    fabsf(angulo);


  // Alvo praticamente centralizado
  if (
    anguloAbsoluto <=
    ANGULO_CENTRAL_GRAUS
  ) {

    frenteDiferencial(
      VELOCIDADE_ATAQUE,
      VELOCIDADE_ATAQUE
    );

    return;
  }


  float intensidade =
    anguloAbsoluto /
    ANGULO_CORRECAO_MAX_GRAUS;


  intensidade =
    constrain(
      intensidade,
      0.0f,
      1.0f
    );


  int velocidadeInterna =
    VELOCIDADE_ATAQUE -
    (
      (
        VELOCIDADE_ATAQUE -
        VELOCIDADE_CURVA_ATAQUE
      )
      *
      intensidade
    );


  // Oponente à esquerda
  if (angulo < 0.0f) {

    frenteDiferencial(
      velocidadeInterna,
      VELOCIDADE_ATAQUE
    );

    return;
  }


  // Oponente à direita
  frenteDiferencial(
    VELOCIDADE_ATAQUE,
    velocidadeInterna
  );
}


// =====================================================
// ATAQUE AO OPONENTE
// =====================================================

void atacarOponente() {

  // A prioridade da borda já é tratada
  // globalmente no começo do loop().


  // Nenhum oponente
  if (!oponenteDetectado) {

    pararMotores();

    estadoAtual =
      RobotState::TARGET_LOST;

    inicioEstadoMs =
      millis();

    return;
  }


  // Triangulação disponível
  if (
    modoRastreamento ==
    OpponentTrackingMode::TRIANGULATED
  ) {

    atacarComAngulo(
      anguloOponenteGraus
    );

    return;
  }


  // Somente sensor esquerdo
  if (
    modoRastreamento ==
    OpponentTrackingMode::LEFT_ONLY
  ) {

    frenteDiferencial(
      VELOCIDADE_CURVA_ATAQUE,
      VELOCIDADE_ATAQUE
    );

    return;
  }


  // Somente sensor direito
  if (
    modoRastreamento ==
    OpponentTrackingMode::RIGHT_ONLY
  ) {

    frenteDiferencial(
      VELOCIDADE_ATAQUE,
      VELOCIDADE_CURVA_ATAQUE
    );

    return;
  }


  // Dois sensores, mas triangulação inválida
  if (
    modoRastreamento ==
    OpponentTrackingMode::BOTH
  ) {

    float diferenca =
      distanciaEsquerda -
      distanciaDireita;


    // Distâncias semelhantes = reto
    if (
      fabsf(diferenca) <=
      DIFERENCA_DISTANCIA_CENTRO_CM
    ) {

      frenteDiferencial(
        VELOCIDADE_ATAQUE,
        VELOCIDADE_ATAQUE
      );

      return;
    }


    // Adversário mais próximo do lado esquerdo
    if (
      distanciaEsquerda <
      distanciaDireita
    ) {

      frenteDiferencial(
        VELOCIDADE_CURVA_ATAQUE,
        VELOCIDADE_ATAQUE
      );

      return;
    }


    // Adversário mais próximo do lado direito
    frenteDiferencial(
      VELOCIDADE_ATAQUE,
      VELOCIDADE_CURVA_ATAQUE
    );

    return;
  }


  // Estado inesperado
  pararMotores();
}


// =====================================================
// CONFIGURAÇÃO DOS SENSORES
// =====================================================

void configurarSensores() {

  pinMode(
    SENSOR_BORDA_ESQUERDO,
    INPUT
  );

  pinMode(
    SENSOR_BORDA_DIREITO,
    INPUT
  );


  pinMode(
    HC_ESQ_TRIG,
    OUTPUT
  );

  pinMode(
    HC_ESQ_ECHO,
    INPUT
  );


  pinMode(
    HC_DIR_TRIG,
    OUTPUT
  );

  pinMode(
    HC_DIR_ECHO,
    INPUT
  );


  digitalWrite(
    HC_ESQ_TRIG,
    LOW
  );

  digitalWrite(
    HC_DIR_TRIG,
    LOW
  );
}


// =====================================================
// LEITURA DOS SENSORES DE BORDA
// =====================================================

void atualizarSensoresBorda() {

  bordaEsquerda =
    (
      digitalRead(
        SENSOR_BORDA_ESQUERDO
      )
      ==
      NIVEL_BORDA
    );


  bordaDireita =
    (
      digitalRead(
        SENSOR_BORDA_DIREITO
      )
      ==
      NIVEL_BORDA
    );


  bordaDetectada =
    bordaEsquerda ||
    bordaDireita;
}
// =====================================================
// VALIDAÇÃO AUTOMÁTICA DOS GPIOs
// =====================================================


// Informações de cada GPIO usado pelo robô.
struct ConfiguracaoGPIO {

  const char* nome;

  uint8_t gpio;

  // true  = precisa funcionar como saída
  // false = será usado somente como entrada
  bool precisaSaida;
};


// =====================================================
// GPIO EXISTE NO ESP32 DEVKIT V1?
// =====================================================

bool gpioExiste(uint8_t gpio) {

  // GPIO 0 até 19 existem,
  // exceto algumas restrições tratadas separadamente.
  if (gpio <= 19) {
    return true;
  }


  // GPIO20 não existe no ESP32 clássico.


  // GPIO21 até GPIO23
  if (
    gpio >= 21 &&
    gpio <= 23
  ) {
    return true;
  }


  // GPIO24 não existe.


  // GPIO25 até GPIO27
  if (
    gpio >= 25 &&
    gpio <= 27
  ) {
    return true;
  }


  // GPIO28 até GPIO31 não existem.


  // GPIO32 até GPIO39
  if (
    gpio >= 32 &&
    gpio <= 39
  ) {
    return true;
  }


  return false;
}


// =====================================================
// GPIO RESERVADO PARA FLASH
// =====================================================

bool gpioReservadoFlash(uint8_t gpio) {

  return (
    gpio >= 6 &&
    gpio <= 11
  );
}


// =====================================================
// GPIO SOMENTE DE ENTRADA
// =====================================================
//
// ESP32 clássico:
// GPIO34, 35, 36, 37, 38 e 39
// não podem ser usados como saída.
// =====================================================

bool gpioSomenteEntrada(uint8_t gpio) {

  return (
    gpio >= 34 &&
    gpio <= 39
  );
}


// =====================================================
// GPIO DE BOOT / STRAPPING
// =====================================================
//
// Podem ser usados durante execução,
// mas o nível presente durante o boot
// precisa ser considerado.
// =====================================================

bool gpioStrapping(uint8_t gpio) {

  return (
    gpio == 0  ||
    gpio == 2  ||
    gpio == 5  ||
    gpio == 12 ||
    gpio == 15
  );
}


// =====================================================
// GPIO USADO PELA SERIAL PRINCIPAL
// =====================================================
//
// Não bloqueamos.
// Apenas mostramos aviso caso sejam usados futuramente.
// =====================================================

bool gpioSerialPrincipal(uint8_t gpio) {

  return (
    gpio == 1 ||
    gpio == 3
  );
}


// =====================================================
// VALIDAÇÃO COMPLETA
// =====================================================

bool validarPinos() {

  const ConfiguracaoGPIO pinos[] = {

    // -----------------------------------------
    // MOTORES
    // -----------------------------------------

    {
      "MOTOR_ESQ_RPWM",
      MOTOR_ESQ_RPWM,
      true
    },

    {
      "MOTOR_ESQ_LPWM",
      MOTOR_ESQ_LPWM,
      true
    },

    {
      "MOTOR_DIR_RPWM",
      MOTOR_DIR_RPWM,
      true
    },

    {
      "MOTOR_DIR_LPWM",
      MOTOR_DIR_LPWM,
      true
    },


    // -----------------------------------------
    // SENSORES DE BORDA
    // -----------------------------------------

    {
      "SENSOR_BORDA_ESQUERDO",
      SENSOR_BORDA_ESQUERDO,
      false
    },

    {
      "SENSOR_BORDA_DIREITO",
      SENSOR_BORDA_DIREITO,
      false
    },


    // -----------------------------------------
    // HC-SR04 ESQUERDO
    // -----------------------------------------

    {
      "HC_ESQ_TRIG",
      HC_ESQ_TRIG,
      true
    },

    {
      "HC_ESQ_ECHO",
      HC_ESQ_ECHO,
      false
    },


    // -----------------------------------------
    // HC-SR04 DIREITO
    // -----------------------------------------

    {
      "HC_DIR_TRIG",
      HC_DIR_TRIG,
      true
    },

    {
      "HC_DIR_ECHO",
      HC_DIR_ECHO,
      false
    }
  };


  constexpr size_t QUANTIDADE_PINOS =
    sizeof(pinos) /
    sizeof(pinos[0]);


  bool configuracaoValida = true;


  // ===================================================
  // CABEÇALHO
  // ===================================================

  if (DEBUG) {

    Serial.println();

    Serial.println(
      "=============================="
    );

    Serial.println(
      " VALIDACAO DOS GPIOs"
    );

    Serial.println(
      "=============================="
    );
  }


  // ===================================================
  // VALIDA CADA GPIO
  // ===================================================

  for (
    size_t i = 0;
    i < QUANTIDADE_PINOS;
    i++
  ) {

    uint8_t gpio =
      pinos[i].gpio;


    // -------------------------------------------------
    // GPIO INEXISTENTE
    // -------------------------------------------------

    if (!gpioExiste(gpio)) {

      configuracaoValida = false;


      if (DEBUG) {

        Serial.print("[ERRO] ");

        Serial.print(
          pinos[i].nome
        );

        Serial.print(
          ": GPIO "
        );

        Serial.print(gpio);

        Serial.println(
          " nao existe no ESP32."
        );
      }


      continue;
    }


    // -------------------------------------------------
    // GPIO RESERVADO PARA FLASH
    // -------------------------------------------------

    if (gpioReservadoFlash(gpio)) {

      configuracaoValida = false;


      if (DEBUG) {

        Serial.print("[ERRO] ");

        Serial.print(
          pinos[i].nome
        );

        Serial.print(
          ": GPIO "
        );

        Serial.print(gpio);

        Serial.println(
          " e reservado para memoria flash."
        );
      }


      continue;
    }


    // -------------------------------------------------
    // GPIO SOMENTE DE ENTRADA USADO COMO SAÍDA
    // -------------------------------------------------

    if (
      pinos[i].precisaSaida &&
      gpioSomenteEntrada(gpio)
    ) {

      configuracaoValida = false;


      if (DEBUG) {

        Serial.print("[ERRO] ");

        Serial.print(
          pinos[i].nome
        );

        Serial.print(
          ": GPIO "
        );

        Serial.print(gpio);

        Serial.println(
          " e somente entrada."
        );
      }


      continue;
    }


    // -------------------------------------------------
    // STRAPPING PIN
    // -------------------------------------------------

    if (gpioStrapping(gpio)) {

      if (DEBUG) {

        Serial.print("[AVISO] ");

        Serial.print(
          pinos[i].nome
        );

        Serial.print(
          ": GPIO "
        );

        Serial.print(gpio);

        Serial.println(
          " e um pino de boot/strapping."
        );
      }
    }


    // -------------------------------------------------
    // UART0
    // -------------------------------------------------

    if (gpioSerialPrincipal(gpio)) {

      if (DEBUG) {

        Serial.print("[AVISO] ");

        Serial.print(
          pinos[i].nome
        );

        Serial.print(
          ": GPIO "
        );

        Serial.print(gpio);

        Serial.println(
          " e usado pela Serial principal."
        );
      }
    }
  }


  // ===================================================
  // PROCURA GPIOs REPETIDOS
  // ===================================================

  for (
    size_t i = 0;
    i < QUANTIDADE_PINOS;
    i++
  ) {

    for (
      size_t j = i + 1;
      j < QUANTIDADE_PINOS;
      j++
    ) {

      if (
        pinos[i].gpio ==
        pinos[j].gpio
      ) {

        configuracaoValida = false;


        if (DEBUG) {

          Serial.print(
            "[ERRO] GPIO "
          );

          Serial.print(
            pinos[i].gpio
          );

          Serial.print(
            " esta repetido: "
          );

          Serial.print(
            pinos[i].nome
          );

          Serial.print(
            " <-> "
          );

          Serial.println(
            pinos[j].nome
          );
        }
      }
    }
  }


  // ===================================================
  // RESULTADO FINAL
  // ===================================================

  if (DEBUG) {

    Serial.println();


    if (configuracaoValida) {

      Serial.println(
        "[OK] Configuracao dos GPIOs valida."
      );

    } else {

      Serial.println(
        "[FALHA] Configuracao dos GPIOs invalida."
      );
    }


    Serial.println(
      "=============================="
    );

    Serial.println();
  }


  return configuracaoValida;
}
// =====================================================
// SISTEMA DE DEBUG
// =====================================================


// =====================================================
// NOME DO ESTADO PRINCIPAL
// =====================================================

const char* nomeEstadoRobot(
  RobotState estado
) {

  switch (estado) {

    case RobotState::STARTUP_DELAY:
      return "STARTUP_DELAY";

    case RobotState::INITIAL_ADVANCE:
      return "INITIAL_ADVANCE";

    case RobotState::EDGE_RECOVERY:
      return "EDGE_RECOVERY";

    case RobotState::SEARCH_TURN_RIGHT:
      return "SEARCH_TURN_RIGHT";

    case RobotState::SEARCH_TURN_LEFT:
      return "SEARCH_TURN_LEFT";

    case RobotState::SEARCH_FACE_BACK:
      return "SEARCH_FACE_BACK";

    case RobotState::ATTACK:
      return "ATTACK";

    case RobotState::TARGET_LOST:
      return "TARGET_LOST";

    case RobotState::SAFE_STOP:
      return "SAFE_STOP";

    default:
      return "UNKNOWN";
  }
}


// =====================================================
// NOME DA FASE DE RECUPERAÇÃO
// =====================================================

const char* nomeEstadoRecuperacao(
  EdgeRecoveryState estado
) {

  switch (estado) {

    case EdgeRecoveryState::STOP:
      return "STOP";

    case EdgeRecoveryState::BACKWARD:
      return "BACKWARD";

    case EdgeRecoveryState::TURN:
      return "TURN";

    case EdgeRecoveryState::FINISHED:
      return "FINISHED";

    default:
      return "UNKNOWN";
  }
}


// =====================================================
// NOME DO MODO DE RASTREAMENTO
// =====================================================

const char* nomeModoRastreamento(
  OpponentTrackingMode modo
) {

  switch (modo) {

    case OpponentTrackingMode::NONE:
      return "NONE";

    case OpponentTrackingMode::LEFT_ONLY:
      return "LEFT_ONLY";

    case OpponentTrackingMode::RIGHT_ONLY:
      return "RIGHT_ONLY";

    case OpponentTrackingMode::BOTH:
      return "BOTH";

    case OpponentTrackingMode::TRIANGULATED:
      return "TRIANGULATED";

    default:
      return "UNKNOWN";
  }
}


// =====================================================
// IMPRIME UMA DISTÂNCIA
// =====================================================
//
// Se a leitura for inválida,
// imprime --
// =====================================================

void imprimirDistancia(
  float distancia,
  uint32_t momentoLeitura
) {

  if (
    !leituraAindaValida(
      distancia,
      momentoLeitura
    )
  ) {

    Serial.print("--");

    return;
  }


  Serial.print(
    distancia,
    1
  );

  Serial.print("cm");
}


// =====================================================
// DEBUG PERIÓDICO
// =====================================================

void imprimirDebug() {

  // DEBUG desligado:
  // não faz absolutamente nada.
  if (!DEBUG) {
    return;
  }


  uint32_t agoraMs =
    millis();


  // Evita inundar o Monitor Serial.
  if (
    agoraMs - ultimoDebugMs <
    DEBUG_INTERVAL_MS
  ) {
    return;
  }


  ultimoDebugMs =
    agoraMs;


  // ===================================================
  // ESTADO PRINCIPAL
  // ===================================================

  Serial.println();

  Serial.println(
    "----------------------------------------"
  );


  Serial.print(
    "[ESTADO] "
  );

  Serial.println(
    nomeEstadoRobot(
      estadoAtual
    )
  );


  // ===================================================
  // RECUPERAÇÃO DE BORDA
  // ===================================================

  if (
    estadoAtual ==
    RobotState::EDGE_RECOVERY
  ) {

    Serial.print(
      "[RECUPERACAO] "
    );

    Serial.print(
      nomeEstadoRecuperacao(
        estadoRecuperacao
      )
    );


    Serial.print(
      " | Giro: "
    );


    if (recuperarGirandoDireita) {

      Serial.println(
        "DIREITA"
      );

    } else {

      Serial.println(
        "ESQUERDA"
      );
    }
  }


  // ===================================================
  // SENSORES DE BORDA
  // ===================================================

  Serial.print(
    "[BORDA] E="
  );

  Serial.print(
    bordaEsquerda ? 1 : 0
  );


  Serial.print(
    " D="
  );

  Serial.print(
    bordaDireita ? 1 : 0
  );


  Serial.print(
    " | Detectada="
  );

  Serial.println(
    bordaDetectada ? "SIM" : "NAO"
  );


  // ===================================================
  // HC-SR04
  // ===================================================

  Serial.print(
    "[HC] E="
  );


  imprimirDistancia(
    distanciaEsquerda,
    ultimaLeituraEsquerdaMs
  );


  Serial.print(
    " | D="
  );


  imprimirDistancia(
    distanciaDireita,
    ultimaLeituraDireitaMs
  );


  Serial.println();


  // ===================================================
  // OPONENTE
  // ===================================================

  Serial.print(
    "[OPONENTE] Detectado="
  );

  Serial.print(
    oponenteDetectado ?
    "SIM" :
    "NAO"
  );


  Serial.print(
    " | E="
  );

  Serial.print(
    oponenteEsquerda ? 1 : 0
  );


  Serial.print(
    " D="
  );

  Serial.print(
    oponenteDireita ? 1 : 0
  );


  Serial.print(
    " | Modo="
  );

  Serial.println(
    nomeModoRastreamento(
      modoRastreamento
    )
  );


  // ===================================================
  // TRIANGULAÇÃO
  // ===================================================

  Serial.print(
    "[POSICAO] "
  );


  if (triangulacaoValida) {

    Serial.print(
      "X="
    );

    Serial.print(
      oponenteXcm,
      1
    );


    Serial.print(
      "cm | Y="
    );

    Serial.print(
      oponenteYcm,
      1
    );


    Serial.print(
      "cm | Dist="
    );

    Serial.print(
      distanciaOponenteCentroCm,
      1
    );


    Serial.print(
      "cm | Ang="
    );

    Serial.print(
      anguloOponenteGraus,
      1
    );

    Serial.println(
      " graus"
    );

  } else {

    Serial.println(
      "SEM TRIANGULACAO"
    );
  }


  // ===================================================
  // BUSCA
  // ===================================================

  Serial.print(
    "[BUSCA] Ciclos="
  );

  Serial.println(
    ciclosBusca
  );


  Serial.println(
    "----------------------------------------"
  );
}
// ===================================================
// SEGURANÇA DA BORDA
// ===================================================

Serial.print(
  "[SEGURANCA] Recuperacao="
);

Serial.print(
  tentativasRecuperacao
);

Serial.print(
  "/"
);

Serial.println(
  MAX_TENTATIVAS_RECUPERACAO
);
// =====================================================
// ESCOLHE O SENTIDO DA RECUPERAÇÃO
// =====================================================

void escolherDirecaoRecuperacao() {

  // Borda somente na esquerda:
  // gira para direita.
  if (
    bordaEsquerda &&
    !bordaDireita
  ) {

    recuperarGirandoDireita = true;
    return;
  }


  // Borda somente na direita:
  // gira para esquerda.
  if (
    bordaDireita &&
    !bordaEsquerda
  ) {

    recuperarGirandoDireita = false;
    return;
  }


  // Os dois sensores estão sobre a borda.
  //
  // Alterna o sentido entre as tentativas
  // para evitar repetir uma estratégia
  // que não está funcionando.
  recuperarGirandoDireita =
    (tentativasRecuperacao % 2 != 0);
}
void setup() {

  // ===================================================
  // SERIAL
  // ===================================================

  if (DEBUG) {
    Serial.begin(SERIAL_BAUD_RATE);
  }

    // ===================================================
  // VALIDAÇÃO DOS GPIOs
  // ===================================================

  configuracaoHardwareValida =
    validarPinos();


  // Se houver qualquer erro grave nos GPIOs,
  // não inicializa motores nem sensores.
  if (!configuracaoHardwareValida) {

    estadoAtual =
      RobotState::SAFE_STOP;


    if (DEBUG) {

      Serial.println(
        "[SEGURANCA] Inicializacao cancelada."
      );
    }


    return;
  }
  // ===================================================
  // MOTORES
  // ===================================================

  // Garante nível baixo antes de ativar o PWM.
  pinMode(MOTOR_ESQ_RPWM, OUTPUT);
  pinMode(MOTOR_ESQ_LPWM, OUTPUT);
  pinMode(MOTOR_DIR_RPWM, OUTPUT);
  pinMode(MOTOR_DIR_LPWM, OUTPUT);

  digitalWrite(MOTOR_ESQ_RPWM, LOW);
  digitalWrite(MOTOR_ESQ_LPWM, LOW);
  digitalWrite(MOTOR_DIR_RPWM, LOW);
  digitalWrite(MOTOR_DIR_LPWM, LOW);


  // Configura PWM usando Arduino-ESP32 Core 3.x.
  bool pwmOk = true;

  pwmOk &= ledcAttach(
    MOTOR_ESQ_RPWM,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  pwmOk &= ledcAttach(
    MOTOR_ESQ_LPWM,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  pwmOk &= ledcAttach(
    MOTOR_DIR_RPWM,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  pwmOk &= ledcAttach(
    MOTOR_DIR_LPWM,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );


  // Se algum PWM falhar, não permite movimento.
  if (!pwmOk) {

    pararMotores();

    estadoAtual =
      RobotState::SAFE_STOP;

    if (DEBUG) {
      Serial.println(
        "[ERRO] Falha ao configurar PWM."
      );
    }

    return;
  }


  // Motores começam obrigatoriamente parados.
  pararMotores();


  // ===================================================
  // SENSORES
  // ===================================================

  configurarSensores();

  atualizarSensoresBorda();


  // Permite que a primeira medição ultrassônica
  // comece rapidamente.
  ultimaMedicaoUltrassomUs =
    micros() - ULTRASONIC_INTERVAL_US;


  // ===================================================
  // ATRASO ALEATÓRIO
  // ===================================================

  tempoStartupMs =
    esp_random() %
    (STARTUP_DELAY_MAX_MS + 1U);


  inicioEstadoMs = millis();

  estadoAtual =
    RobotState::STARTUP_DELAY;


  if (DEBUG) {

    Serial.println();
    Serial.println(
      "=============================="
    );

    Serial.println(
      "   CARRINHO SUMO INICIADO"
    );

    Serial.println(
      "=============================="
    );

    Serial.print(
      "Tempo inicial: "
    );

    Serial.print(
      tempoStartupMs
    );

    Serial.println(
      " ms"
    );
  }
}
void loop() {

  // ===================================================
  // BLOQUEIO POR CONFIGURAÇÃO INVÁLIDA
  // ===================================================

  if (!configuracaoHardwareValida) {
    return;
  }


  uint32_t agoraMs = millis();


  // ===================================================
  // ATUALIZA OS SENSORES O TEMPO TODO
  // ===================================================

  atualizarSensoresBorda();

  atualizarUltrassonicos();

  atualizarRastreamentoOponente();


  // ===================================================
  // PRIORIDADE ABSOLUTA: BORDA
  // ===================================================
  //
  // Não reinicia recuperação se já estivermos
  // recuperando.
  //
  // Também ignora durante STARTUP_DELAY,
  // pois o robô está parado.
  // ===================================================

 if (
  bordaDetectada &&
  estadoAtual != RobotState::STARTUP_DELAY &&
  estadoAtual != RobotState::EDGE_RECOVERY &&
  estadoAtual != RobotState::SAFE_STOP
) {

  // ===================================================
  // PARA IMEDIATAMENTE
  // ===================================================

  pararMotores();


  // ===================================================
  // PRIMEIRA TENTATIVA DESTE EVENTO
  // ===================================================

  tentativasRecuperacao = 1;


  // Decide para qual lado girar.
  escolherDirecaoRecuperacao();


  // ===================================================
  // INICIA A RECUPERAÇÃO
  // ===================================================

  estadoRecuperacao =
    EdgeRecoveryState::STOP;

  inicioRecuperacaoMs =
    agoraMs;

  estadoAtual =
    RobotState::EDGE_RECOVERY;


  if (DEBUG) {

    Serial.print(
      "[BORDA] Recuperacao tentativa "
    );

    Serial.print(
      tentativasRecuperacao
    );

    Serial.print(
      "/"
    );

    Serial.println(
      MAX_TENTATIVAS_RECUPERACAO
    );
  }
}

  // ===================================================
  // MÁQUINA DE ESTADOS PRINCIPAL
  // ===================================================

  switch (estadoAtual) {


    // =================================================
    // 1. ESPERA ALEATÓRIA DE 0 A 3 SEGUNDOS
    // =================================================

    case RobotState::STARTUP_DELAY:

      pararMotores();


      if (
        agoraMs - inicioEstadoMs >=
        tempoStartupMs
      ) {

        estadoAtual =
          RobotState::INITIAL_ADVANCE;

        inicioEstadoMs =
          agoraMs;


        if (DEBUG) {

          Serial.println(
            "[ESTADO] INITIAL_ADVANCE"
          );
        }
      }

      break;


    // =================================================
    // 2. AVANÇA ATÉ ENCONTRAR A BORDA
    // =================================================

    case RobotState::INITIAL_ADVANCE:

      frente(
        VELOCIDADE_INICIAL
      );

      // A detecção da borda é tratada
      // no início do loop.

      break;


    // =================================================
    // 3. RECUPERAÇÃO DA BORDA
    // =================================================

    case RobotState::EDGE_RECOVERY:


      switch (estadoRecuperacao) {


        // ---------------------------------------------
        // PARA PRIMEIRO
        // ---------------------------------------------

        case EdgeRecoveryState::STOP:

          pararMotores();


          if (
            agoraMs -
            inicioRecuperacaoMs >=
            EDGE_STOP_MS
          ) {

            estadoRecuperacao =
              EdgeRecoveryState::BACKWARD;

            inicioRecuperacaoMs =
              agoraMs;
          }

          break;


        // ---------------------------------------------
        // DÁ RÉ
        // ---------------------------------------------

        case EdgeRecoveryState::BACKWARD:

          re(
            VELOCIDADE_FUGA_BORDA
          );


          if (
            agoraMs -
            inicioRecuperacaoMs >=
            EDGE_ESCAPE_MS
          ) {

            pararMotores();

            estadoRecuperacao =
              EdgeRecoveryState::TURN;

            inicioRecuperacaoMs =
              agoraMs;
          }

          break;


        // ---------------------------------------------
        // VIRA PARA O INTERIOR
        // ---------------------------------------------

        case EdgeRecoveryState::TURN:


          if (recuperarGirandoDireita) {

            direita(
              VELOCIDADE_GIRO
            );

          } else {

            esquerda(
              VELOCIDADE_GIRO
            );
          }


          if (
            agoraMs -
            inicioRecuperacaoMs >=
            EDGE_TURN_MS
          ) {

            pararMotores();

            estadoRecuperacao =
              EdgeRecoveryState::FINISHED;

            inicioRecuperacaoMs =
              agoraMs;
          }

          break;


        // ---------------------------------------------
        // RECUPERAÇÃO TERMINADA
        // ---------------------------------------------

        case EdgeRecoveryState::FINISHED:

  // Mantém parado enquanto confirma
  // se realmente saiu da borda.
  pararMotores();


  // -------------------------------------------------
  // AGUARDA UM PEQUENO PERÍODO DE CONFIRMAÇÃO
  // -------------------------------------------------

  if (
    agoraMs -
    inicioRecuperacaoMs <
    EDGE_CLEAR_CONFIRM_MS
  ) {

    break;
  }


  // -------------------------------------------------
  // BORDA LIBERADA
  // -------------------------------------------------

  if (!bordaDetectada) {

    if (DEBUG) {

      Serial.print(
        "[BORDA] Recuperacao concluida em "
      );

      Serial.print(
        tentativasRecuperacao
      );

      Serial.println(
        " tentativa(s)."
      );
    }


    // Recuperação foi bem-sucedida.
    tentativasRecuperacao = 0;


    // Começa a procurar o oponente.
    estadoAtual =
      RobotState::SEARCH_TURN_RIGHT;

    inicioEstadoMs =
      agoraMs;


    if (DEBUG) {

      Serial.println(
        "[ESTADO] SEARCH_TURN_RIGHT"
      );
    }


    break;
  }


  // -------------------------------------------------
  // AINDA ESTÁ NA BORDA
  // -------------------------------------------------

  if (
    tentativasRecuperacao >=
    MAX_TENTATIVAS_RECUPERACAO
  ) {

    // Não conseguiu escapar dentro do limite.
    pararMotores();

    estadoAtual =
      RobotState::SAFE_STOP;


    if (DEBUG) {

      Serial.println();

      Serial.println(
        "[SEGURANCA] LIMITE DE RECUPERACAO ATINGIDO!"
      );

      Serial.print(
        "[SEGURANCA] Tentativas: "
      );

      Serial.println(
        tentativasRecuperacao
      );

      Serial.println(
        "[SEGURANCA] Entrando em SAFE_STOP."
      );
    }


    break;
  }


  // -------------------------------------------------
  // NOVA TENTATIVA
  // -------------------------------------------------

  tentativasRecuperacao++;


  // Escolhe novamente a direção.
  //
  // Caso os dois sensores continuem ativos,
  // a função alternará esquerda/direita.
  escolherDirecaoRecuperacao();


  estadoRecuperacao =
    EdgeRecoveryState::STOP;

  inicioRecuperacaoMs =
    agoraMs;


  if (DEBUG) {

    Serial.print(
      "[BORDA] Ainda detectada. Nova tentativa "
    );

    Serial.print(
      tentativasRecuperacao
    );

    Serial.print(
      "/"
    );

    Serial.println(
      MAX_TENTATIVAS_RECUPERACAO
    );
  }


  break;
      }

      break;


    // =================================================
    // 4. PROCURA 90° PARA DIREITA
    // =================================================

    case RobotState::SEARCH_TURN_RIGHT:


      // Encontrou adversário.
      if (oponenteDetectado) {

        pararMotores();

        estadoAtual =
          RobotState::ATTACK;

        inicioEstadoMs =
          agoraMs;

        break;
      }


      direita(
        VELOCIDADE_BUSCA
      );


      if (
        agoraMs - inicioEstadoMs >=
        TURN_90_MS
      ) {

        pararMotores();

        estadoAtual =
          RobotState::SEARCH_TURN_LEFT;

        inicioEstadoMs =
          agoraMs;
      }

      break;


    // =================================================
    // 5. PROCURA 180° PARA ESQUERDA
    // =================================================

    case RobotState::SEARCH_TURN_LEFT:


      if (oponenteDetectado) {

        pararMotores();

        estadoAtual =
          RobotState::ATTACK;

        inicioEstadoMs =
          agoraMs;

        break;
      }


      esquerda(
        VELOCIDADE_BUSCA
      );


      if (
        agoraMs - inicioEstadoMs >=
        TURN_180_MS
      ) {

        pararMotores();

        estadoAtual =
          RobotState::SEARCH_FACE_BACK;

        inicioEstadoMs =
          agoraMs;
      }

      break;


    // =================================================
    // 6. MAIS 90° PARA ESQUERDA
    //
    // Deixa o robô voltado para trás da
    // orientação em que começou a busca.
    // =================================================

    case RobotState::SEARCH_FACE_BACK:


      if (oponenteDetectado) {

        pararMotores();

        estadoAtual =
          RobotState::ATTACK;

        inicioEstadoMs =
          agoraMs;

        break;
      }


      esquerda(
        VELOCIDADE_BUSCA
      );


      if (
        agoraMs - inicioEstadoMs >=
        TURN_90_MS
      ) {

        pararMotores();

        ciclosBusca++;


        // Reinicia a sequência.
        estadoAtual =
          RobotState::SEARCH_TURN_RIGHT;

        inicioEstadoMs =
          agoraMs;
      }

      break;


    // =================================================
    // 7. ATAQUE
    // =================================================

    case RobotState::ATTACK:

      atacarOponente();

      break;


    // =================================================
    // 8. ADVERSÁRIO PERDIDO
    // =================================================

    case RobotState::TARGET_LOST:

      pararMotores();


      // Se reaparecer durante a tolerância,
      // volta imediatamente ao ataque.
      if (oponenteDetectado) {

        estadoAtual =
          RobotState::ATTACK;

        inicioEstadoMs =
          agoraMs;

        break;
      }


      // Se realmente perdeu o alvo,
      // volta para a busca.
      if (
        agoraMs - inicioEstadoMs >=
        TARGET_LOST_MS
      ) {

        estadoAtual =
          RobotState::SEARCH_TURN_RIGHT;

        inicioEstadoMs =
          agoraMs;
      }

      break;


    // =================================================
    // 9. PARADA DE SEGURANÇA
    // =================================================

    case RobotState::SAFE_STOP:

      pararMotores();

      break;
  }

  // ===================================================
  // DEBUG
  // ===================================================

  imprimirDebug();
}