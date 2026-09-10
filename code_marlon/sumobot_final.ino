/*
  =====================================================================
  SUMOBOT - CODIGO COMPLETO DA DISPUTA
  =====================================================================
  Sequencia:
    1) Liga -> espera ate 3s.
    2) Da re ate o sensor de re (tras) achar a borda branca -> avanca
       um pouco pra sair de cima da linha -> para.
    3) A partir dai, entra em operacao continua:
         - Se o sensor de linha da FRENTE ver branco a qualquer
           momento -> re instantanea de ~2s -> gira 180 pra ficar de
           frente pro centro -> volta a procurar. Isso tem prioridade
           sobre tudo o mais, sempre.
         - Se os dois sensores de distancia virem o adversario ao
           mesmo tempo -> ele esta em linha reta na frente -> avanca
           com tudo.
         - Se so um sensor de distancia ver -> curva pra aquele lado
           avancando ao mesmo tempo (se alinha e ja fecha distancia).
         - Se nenhum ver -> varre de um lado pro outro (sem dar volta
           completa) procurando.

  ===================== LEIA ANTES DE CONFIAR NISSO =====================
  1) NIVEL_BRANCO (linha ~50): o comportamento LOW/HIGH dos sensores de
     linha varia de modulo pra modulo e NUNCA foi testado nessa nossa
     conversa. Teste em cima do preto e do branco reais da arena antes
     da disputa - se estiver invertido, o robo faz o oposto do que
     deveria (anda PRA a borda em vez de fugir).
  2) TEMPO_MEIO_GIRO_MS e TEMPO_GIRO_180_MS (linha ~60) sao APROXIMACOES
     por tempo, calibradas no chute - nao ha giroscopio nem encoder pra
     medir angulo de verdade. Cronometre visualmente quanto tempo o seu
     robo leva pra girar 90 e 180 graus na velocidade escolhida e ajuste
     esses dois valores antes de confiar no giro de retomada.
  3) TRIM_ESQ / TRIM_DIR (linha ~66): usa os mesmos valores que voce ja
     calibrou nos testes anteriores pra ele andar reto avancando.
  4) Teste PRIMEIRO com o robo apoiado (rodas no ar) rodando so o Serial
     Monitor, conferindo se os prints de sensor de linha fazem sentido
     (0 sobre o preto, 1 sobre o branco, ou o contrario - so o teste
     real confirma).

  Pinagem igual ao guia de ligacoes:
    Motor Esquerdo - velocidade frente: GPIO27 / velocidade re: GPIO26
    Motor Direito  - velocidade frente: GPIO25 / velocidade re: GPIO33
    Sensor de Re Esquerdo  (TCRT5000) - sinal: GPIO13
    Sensor de Re Direito   (TCRT5000) - sinal: GPIO32
    Sensor de Linha Frontal (3 canais)
        ponta esquerda - sinal: GPIO34
        centro         - sinal: GPIO35
        ponta direita  - sinal: GPIO14
    Sensor do Adversario Esquerdo (HC-SR04) - dispara: GPIO5  / escuta: GPIO18
    Sensor do Adversario Direito  (HC-SR04) - dispara: GPIO19 / escuta: GPIO23
  =====================================================================
*/

// ---------------------- PINAGEM ----------------------

const int PINO_MOTOR_ESQ_FRENTE = 27;
const int PINO_MOTOR_ESQ_RE     = 26;
const int PINO_MOTOR_DIR_FRENTE = 25;
const int PINO_MOTOR_DIR_RE     = 33;

const int PINO_SENSOR_RE_ESQ = 13;
const int PINO_SENSOR_RE_DIR = 32;

const int PINO_LINHA_FRONTAL_ESQ    = 34;
const int PINO_LINHA_FRONTAL_CENTRO = 35;
const int PINO_LINHA_FRONTAL_DIR    = 14;

const int PINO_ADV_ESQ_TRIG = 5;
const int PINO_ADV_ESQ_ECHO = 18;
const int PINO_ADV_DIR_TRIG = 19;
const int PINO_ADV_DIR_ECHO = 23;

// ---------------------- AJUSTES - LEIA O AVISO NO TOPO DO ARQUIVO ----------------------

// Nivel logico que os sensores de linha retornam em cima do BRANCO.
// NUNCA TESTADO - confirme na pratica antes de confiar.
const int NIVEL_BRANCO = LOW;

const unsigned long TEMPO_DELAY_INICIAL_MS   = 3000; // espera antes de comecar (regra da disputa)
const unsigned long TEMPO_MAX_RE_CALIB_MS    = 4000; // seguranca: se o sensor de re nunca achar a borda
const unsigned long TEMPO_AVANCO_CALIB_MS    = 500;  // avanco curto apos achar a borda de tras na calibracao

const unsigned long TEMPO_RE_BORDA_MS  = 2000; // re ao encontrar a borda da frente (pedido: "uns 2 segundos")
const unsigned long TEMPO_GIRO_180_MS  = 900;  // CALIBRAR: tempo pra girar ~180 graus
const unsigned long TEMPO_MEIO_GIRO_MS = 450;  // CALIBRAR: tempo pra girar ~90 graus (varredura de busca)

const unsigned long PAUSA_ENTRE_SENSORES_MS = 20; // evita um ultrassonico "ouvir" o eco do outro
const int DISTANCIA_DETECCAO_CM = 40; // "enxergo o adversario" dentro desse alcance

const int VELOCIDADE_RE_CALIB        = 150; // 0-255, re inicial de calibracao
const int VELOCIDADE_AVANCO_CALIB    = 130; // avanco curto pos-calibracao
const int VELOCIDADE_RE_BORDA        = 170; // re ao bater na borda da frente
const int VELOCIDADE_GIRO_180        = 150; // giro de retomada apos a borda
const int VELOCIDADE_BUSCA_GIRO      = 130; // varredura procurando o adversario
const int VELOCIDADE_ALINHAR_EXTERNA = 170; // roda de fora ao curvar em direcao ao adversario
const int VELOCIDADE_ALINHAR_INTERNA = 60;  // roda de dentro ao curvar em direcao ao adversario
const int VELOCIDADE_ATAQUE          = 255; // adversario em linha reta na frente

// Ajuste fino (trim) pra andar reto - use os valores ja calibrados nos testes.
const int TRIM_ESQ = 0;
const int TRIM_DIR = 0;

// ---------------------- VARIAVEIS DE ESTADO DA BUSCA ----------------------

bool buscaIniciada = false;
int direcaoBusca = 1; // 1 = um sentido, -1 = o outro
unsigned long marcaTempoBusca = 0;
unsigned long ultimoDebugMs = 0;

// ---------------------- SETUP ----------------------

void setup() {
  Serial.begin(115200);

  pinMode(PINO_MOTOR_ESQ_FRENTE, OUTPUT);
  pinMode(PINO_MOTOR_ESQ_RE, OUTPUT);
  pinMode(PINO_MOTOR_DIR_FRENTE, OUTPUT);
  pinMode(PINO_MOTOR_DIR_RE, OUTPUT);

  pinMode(PINO_SENSOR_RE_ESQ, INPUT);
  pinMode(PINO_SENSOR_RE_DIR, INPUT);
  pinMode(PINO_LINHA_FRONTAL_ESQ, INPUT);
  pinMode(PINO_LINHA_FRONTAL_CENTRO, INPUT);
  pinMode(PINO_LINHA_FRONTAL_DIR, INPUT);

  pinMode(PINO_ADV_ESQ_TRIG, OUTPUT);
  pinMode(PINO_ADV_ESQ_ECHO, INPUT);
  pinMode(PINO_ADV_DIR_TRIG, OUTPUT);
  pinMode(PINO_ADV_DIR_ECHO, INPUT);

  pararRobo();

  Serial.println("Sumobot ligado. Aguardando delay inicial...");
  delay(TEMPO_DELAY_INICIAL_MS);

  // ---- Calibracao: re ate achar a borda de tras ----
  Serial.println("Calibracao: dando re ate achar a borda...");
  moverRobo(-VELOCIDADE_RE_CALIB, -VELOCIDADE_RE_CALIB);

  unsigned long inicioRe = millis();
  while (!detectaBranco(PINO_SENSOR_RE_ESQ) && !detectaBranco(PINO_SENSOR_RE_DIR)) {
    if (millis() - inicioRe >= TEMPO_MAX_RE_CALIB_MS) {
      Serial.println("Timeout de seguranca na re de calibracao (sensor pode nao ter disparado).");
      break;
    }
  }
  pararRobo();
  Serial.println("Borda de tras encontrada. Avancando um pouco pra sair da linha...");

  delay(150);
  moverRobo(VELOCIDADE_AVANCO_CALIB + TRIM_ESQ, VELOCIDADE_AVANCO_CALIB + TRIM_DIR);
  delay(TEMPO_AVANCO_CALIB_MS);
  pararRobo();
  delay(300);

  Serial.println("Calibracao concluida. Procurando o adversario.");
}

// ---------------------- LOOP PRINCIPAL ----------------------

void loop() {
  // 1) PRIORIDADE MAXIMA: nunca sair da arena pela frente.
  bool brancoEsq    = detectaBranco(PINO_LINHA_FRONTAL_ESQ);
  bool brancoCentro = detectaBranco(PINO_LINHA_FRONTAL_CENTRO);
  bool brancoDir    = detectaBranco(PINO_LINHA_FRONTAL_DIR);

  if (brancoEsq || brancoCentro || brancoDir) {
    reagirBordaFrontal();
    return; // recomeca o loop do zero, ja de volta no modo de busca
  }

  // 2) Procurar, se alinhar e atacar o adversario.
  long distEsq = lerDistanciaCm(PINO_ADV_ESQ_TRIG, PINO_ADV_ESQ_ECHO);
  delay(PAUSA_ENTRE_SENSORES_MS);
  long distDir = lerDistanciaCm(PINO_ADV_DIR_TRIG, PINO_ADV_DIR_ECHO);

  bool vejoEsq = (distEsq > 0 && distEsq <= DISTANCIA_DETECCAO_CM);
  bool vejoDir = (distDir > 0 && distDir <= DISTANCIA_DETECCAO_CM);

  if (vejoEsq && vejoDir) {
    // Adversario em linha reta na frente: avanca com tudo.
    buscaIniciada = false;
    moverRobo(VELOCIDADE_ATAQUE + TRIM_ESQ, VELOCIDADE_ATAQUE + TRIM_DIR);

  } else if (vejoEsq) {
    // Adversario a esquerda: curva pra esquerda ja avancando (alinha e fecha distancia).
    buscaIniciada = false;
    moverRobo(VELOCIDADE_ALINHAR_INTERNA, VELOCIDADE_ALINHAR_EXTERNA);

  } else if (vejoDir) {
    // Adversario a direita: curva pra direita ja avancando.
    buscaIniciada = false;
    moverRobo(VELOCIDADE_ALINHAR_EXTERNA, VELOCIDADE_ALINHAR_INTERNA);

  } else {
    // Ninguem a vista: varre de um lado pro outro, nunca dando a volta completa.
    girarBusca();
  }

  debugPeriodico(distEsq, distDir, vejoEsq, vejoDir, brancoEsq, brancoCentro, brancoDir);
}

// ---------------------- MANOBRA DE BORDA (FRENTE) ----------------------

void reagirBordaFrontal() {
  Serial.println("BORDA FRONTAL DETECTADA! Re instantanea.");
  pararRobo();
  moverRobo(-VELOCIDADE_RE_BORDA, -VELOCIDADE_RE_BORDA);
  delay(TEMPO_RE_BORDA_MS);
  pararRobo();
  delay(150);

  Serial.println("Girando ~180 graus para ficar de frente pro centro...");
  moverRobo(VELOCIDADE_GIRO_180, -VELOCIDADE_GIRO_180); // sempre gira no mesmo sentido
  delay(TEMPO_GIRO_180_MS);
  pararRobo();
  delay(150);

  buscaIniciada = false; // garante que a proxima varredura comece do zero
  Serial.println("Retomando a busca do adversario.");
}

// ---------------------- BUSCA (VARREDURA) ----------------------

void girarBusca() {
  unsigned long agora = millis();

  if (!buscaIniciada) {
    buscaIniciada = true;
    marcaTempoBusca = agora;
    direcaoBusca = 1;
  }

  if (agora - marcaTempoBusca >= TEMPO_MEIO_GIRO_MS) {
    direcaoBusca = -direcaoBusca; // troca de lado - nunca da a volta completa
    marcaTempoBusca = agora;
  }

  if (direcaoBusca == 1) {
    moverRobo(VELOCIDADE_BUSCA_GIRO, -VELOCIDADE_BUSCA_GIRO);
  } else {
    moverRobo(-VELOCIDADE_BUSCA_GIRO, VELOCIDADE_BUSCA_GIRO);
  }
}

// ---------------------- SENSORES ----------------------

bool detectaBranco(int pino) {
  return digitalRead(pino) == NIVEL_BRANCO;
}

long lerDistanciaCm(int pinoTrig, int pinoEcho) {
  digitalWrite(pinoTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinoTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinoTrig, LOW);

  long duracaoUs = pulseIn(pinoEcho, HIGH, 30000); // timeout de 30ms
  if (duracaoUs == 0) return -1; // sem eco = nao detectou nada no alcance
  return duracaoUs / 58;
}

// ---------------------- MOTORES ----------------------

void setMotorEsquerdo(int velocidade) {
  velocidade = constrain(velocidade, -255, 255);
  if (velocidade >= 0) {
    analogWrite(PINO_MOTOR_ESQ_FRENTE, velocidade);
    analogWrite(PINO_MOTOR_ESQ_RE, 0);
  } else {
    analogWrite(PINO_MOTOR_ESQ_FRENTE, 0);
    analogWrite(PINO_MOTOR_ESQ_RE, -velocidade);
  }
}

void setMotorDireito(int velocidade) {
  velocidade = constrain(velocidade, -255, 255);
  if (velocidade >= 0) {
    analogWrite(PINO_MOTOR_DIR_FRENTE, velocidade);
    analogWrite(PINO_MOTOR_DIR_RE, 0);
  } else {
    analogWrite(PINO_MOTOR_DIR_FRENTE, 0);
    analogWrite(PINO_MOTOR_DIR_RE, -velocidade);
  }
}

void moverRobo(int velocidadeEsq, int velocidadeDir) {
  setMotorEsquerdo(velocidadeEsq);
  setMotorDireito(velocidadeDir);
}

void pararRobo() {
  moverRobo(0, 0);
}

// ---------------------- DEBUG ----------------------

void debugPeriodico(long distEsq, long distDir, bool vejoEsq, bool vejoDir,
                     bool brancoEsq, bool brancoCentro, bool brancoDir) {
  if (millis() - ultimoDebugMs < 300) return;
  ultimoDebugMs = millis();

  Serial.print("distEsq="); Serial.print(distEsq);
  Serial.print("cm vejoEsq="); Serial.print(vejoEsq);
  Serial.print(" | distDir="); Serial.print(distDir);
  Serial.print("cm vejoDir="); Serial.print(vejoDir);
  Serial.print(" | linhaEsq="); Serial.print(brancoEsq);
  Serial.print(" linhaCentro="); Serial.print(brancoCentro);
  Serial.print(" linhaDir="); Serial.println(brancoDir);
}
