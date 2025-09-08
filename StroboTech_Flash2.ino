#include "flash.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
#include <arduinoFFT.h>
// ==== Pinos ====
#define ENCODER_PIN_A 33
#define ENCODER_PIN_B 32
#define BUTTON_MENU   25
#define BUTTON_SET    13
#define BUTTON_DOUBLE 27
#define BUTTON_HALF   26
#define BUTTON_ENC    17
#define LED_PIN       2        // Pino do LED de alta potência para o estroboscópio/lanterna
#define SENSOR_IR_PIN 4  // Pino do sensor infravermelho para medição de RPM
/* 
GPIOObservações 
21-22 -SDA, SCL 
5,23,18,19 - SPI 
*/
// ==== Display ====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// =================
// ==== Encoder e ADXL ====
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(123);
bool adxlAvailable = false;
//==== Inicialização do medidor de FFT ====
#define SAMPLE_RATE 500  // Hz
#define FFT_SIZE 512     //Resolução da frequência 256, 512 ou 1024
double accBuffer[FFT_SIZE];
double timeBuffer[FFT_SIZE];
ArduinoFFT<double> FFT = ArduinoFFT<double>(accBuffer, timeBuffer, FFT_SIZE, SAMPLE_RATE);
// ==== Menu ====
enum class Mode { HOME,
                  FREQUENCY,
                  RPM,
                  LANTERN,
                  VIBROMETER,
                  TEST,
                  ABOUT,
                  CONFIG,
                  NUM_MODES };
Mode currentMode = Mode::HOME;
Mode selectedMode = Mode::HOME;
bool inMenu = true;
bool inSubmenu = false;
bool inEncoder = false;
// ==== Variáveis do Vibrometro ====
enum class VibroState { VIBRO_HOME,
                        VIBRO_IDLE,
                        VIBRO_CALIB,
                        VIBRO_CONFIG,
                        VIBRO_MEASURE,
                        VIBRO_RESULT };
VibroState vibroState = VibroState::VIBRO_HOME;
// ==== Medição ====
float ax, ay, az;
float aRMS, aPeak, stdDev, vRMS, freqDominant;
float velocitySum = 0;
unsigned long lastSampleTime = 0;
int sampleCount = 0;
bool isMeasuring = false;
bool isCalibrating = false;
unsigned long measureStartTime = 0;
unsigned long measureDuration = 0;
unsigned long calibrationStartTime = 0;
unsigned long calibrationDuration = 0;
int secondsLeft = 0;
int secondsLeftCalib = 0;
float offsetX = 0, offsetY = 0, offsetZ = 0;
//-------------------------------------
// ==== Variáveis de Gravação do tempo de vibração ====
int timeMeasure = 30;
int timeCalib = 15;
// ==== Variáveis do RPM ====
#define TEMPO_LEITURA_RPM 1000  // em milissegundos (1 segundo)
unsigned long ultimaLeitura = 0;
int contagemPulsos = 0;
bool estadoAnterior = LOW;
float rpmValue = 0;
// Valor de RPM calculado
bool Lant_calc = false;
bool TESTE_calc = false;
float TESTE_fpm = 0;

float FreqDeTest = 0;
//-------------------------------------
// Lista de mensagens
String lines[] = {
  "Um agradecimento es-",
  "pecial para todos os",
  "professores do SENAI",
  "que nos incentivaram",
  "e nos orientaram ate",
  "o fim.",
  "Sempre acreditando",
  "mais que nos mesmo.",
  "",
  "Nossos professores:",
  "Evandro Padilha",
  "Renato",
  "Vitor Santarosa",
  "Alex Penteado",
  "Bruno",
  "Fernando",
  "Fabio Camarinha",
  "Gabriela"
};
const int totalLines = sizeof(lines) / sizeof(lines[0]);
int topLineIndex = 0;
// Índice da linha superior visível
int lineShowMsg = 5;
// ==============
// ==== Temporizador ====
struct TimerMicros {
  unsigned long start;
  unsigned long duration;
  void startTimer(int d) {
    duration = d * 1000000UL;
    start = micros();
  }
  bool isExpired() {
    return (micros() - start) >= duration;
  }
  bool isRunning() {
    return (micros() - start) < duration;
  }
};
TimerMicros msgTimer;  //Pode criar quantos TimerMicros for necessário
TimerMicros fpmTest;
// ==== Vetores da Flash ====
std::vector<Config> dadosConfig;
std::vector<Config> lastDadosConfig;
std::vector<Config> displayConfig;
int numIdioma = 1;
String idioma = "PT";
bool saveData = false;
// ==============
// ==== Variáveis do Modo Estroboscópio ====
float fpm = 300;  // Valor inicial de FPM
long STB_lastEncoderPos = 0;
float STB_phaseDegrees = 0.0;
// Ajuste de fase em graus
unsigned long STB_partTime = 1000000;
// Duração de meio ciclo em microssegundos
unsigned long STB_phaseDelayMicros = 0;
// Atraso de fase em microssegundos
bool STB_calc = true;
// Indica se os cálculos precisam ser refeitos
bool STB_firstPulse = true;
// Para aplicar a fase apenas uma vez
bool STB_outputEnabled = false;
// Controla se a saída está ativa
hw_timer_t *timer = NULL;
portMUX_TYPE STB_timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool STB_pulseState = false;
const int minFPM = 30;
// Limite mínimo de FPM
const int maxFPM = 40000;
// Limite máximo de FPM
// ==== Função de interrupção do timer. Alterna o estado do LED e aplica o atraso de fase na primeira chamada. ====
void IRAM_ATTR onTimer() {
  static bool phaseApplied = false;
  portENTER_CRITICAL_ISR(&STB_timerMux);
  if (!STB_outputEnabled) {
    digitalWrite(LED_PIN, LOW);
    portEXIT_CRITICAL_ISR(&STB_timerMux);
    return;
  }
  if (STB_firstPulse && !phaseApplied && STB_phaseDelayMicros > 0) {
    timerAlarm(timer, STB_phaseDelayMicros, true, 0);
    // Atraso de fase
    phaseApplied = true;
  } else {
    STB_pulseState = !STB_pulseState;
    digitalWrite(LED_PIN, STB_pulseState);
    timerAlarm(timer, STB_partTime, true, 0);
    // Próximo intervalo
    STB_firstPulse = false;
    phaseApplied = false;
  }
  portEXIT_CRITICAL_ISR(&STB_timerMux);
}
// Atualiza os valores com base na entrada de FPM. Recalcula os tempos de ciclo e atraso de fase.
void updateValues() {
  if (STB_calc) {
    unsigned long cycleTimeMicros = 60000000UL / fpm;
    STB_partTime = cycleTimeMicros / 2;
    STB_phaseDelayMicros = (STB_phaseDegrees / 360.0) * cycleTimeMicros;
    STB_firstPulse = true;
    STB_calc = false;
  } else if (Lant_calc) {
    unsigned long cycleTimeMicros = 60000000UL / 7200;
    STB_partTime = cycleTimeMicros / 2;
    STB_phaseDelayMicros = (STB_phaseDegrees / 360.0) * cycleTimeMicros;
    STB_firstPulse = true;
  } else if (TESTE_calc) {
    unsigned long cycleTimeMicros = 60000000UL / TESTE_fpm;
    STB_partTime = cycleTimeMicros / 2;
    STB_phaseDelayMicros = (STB_phaseDegrees / 360.0) * cycleTimeMicros;
    STB_firstPulse = true;
  }
}
// ==== Ajusta o FPM multiplicando-o por um fator ====
void adjustFPM(float factor) {
  fpm = constrain(fpm * factor, minFPM, maxFPM);
}
bool checkButtonDebounce(uint8_t pin, bool &lastState, unsigned long &lastDebounceTime, unsigned long debounceDelayMicros = 200000) {
  bool currentState = digitalRead(pin);
  unsigned long now = micros();
  if (currentState == LOW && lastState == HIGH && (now - lastDebounceTime > debounceDelayMicros)) {
    lastDebounceTime = now;
    lastState = currentState;
    return true;  // Botão pressionado com debounce válido
  }
  lastState = currentState;
  return false;
}
//-------------------------------------
// ==== Funçoes do Vibrometro calibrar e medir ====
// Inicia calibração com tempo em segundos
void startCalibration(int durationSeconds) {
  isCalibrating = true;
  calibrationDuration = durationSeconds * 1000UL;
  calibrationStartTime = millis();
  offsetX = 0;
  offsetY = 0;
  offsetZ = 0;
  sampleCount = 0;
  secondsLeftCalib = durationSeconds;
  vibroState = VibroState::VIBRO_CALIB;
  Serial.println("Iniciando calibração...");
}
void updateCalibration() {
  if (!isCalibrating) return;
  unsigned long now = millis();
  unsigned long elapsed = now - calibrationStartTime;
  unsigned long remaining = (calibrationDuration > elapsed) ? (calibrationDuration - elapsed) : 0;
  secondsLeftCalib = remaining / 1000;
  sensors_event_t event;
  accel.getEvent(&event);
  offsetX += event.acceleration.x;
  offsetY += event.acceleration.y;
  offsetZ += event.acceleration.z;
  sampleCount++;
  if (elapsed >= calibrationDuration) {
    offsetX /= sampleCount;
    offsetY /= sampleCount;
    offsetZ /= sampleCount;
    isCalibrating = false;
    vibroState = VibroState::VIBRO_IDLE;
    Serial.println("Calibração concluída!");
    Serial.print("Offset X: ");
    Serial.println(offsetX);
    Serial.print("Offset Y: ");
    Serial.println(offsetY);
    Serial.print("Offset Z: ");
    Serial.println(offsetZ);
  }
}
//vibroState = VibroState::VIBRO_CONFIG;
void startMeasurement(int durationSeconds) {
  isMeasuring = true;
  measureStartTime = millis();
  measureDuration = durationSeconds * 1000UL;
  secondsLeft = durationSeconds;
  sampleCount = 0;
  velocitySum = 0;
  aPeak = 0;
  aRMS = 0;
  stdDev = 0;
  freqDominant = 0;
  lastSampleTime = 0;
  vibroState = VibroState::VIBRO_CONFIG;
  Serial.println("Iniciando medição...");
}
//vibroState = VibroState::VIBRO_MEASURE;
void updateMeasurement() {
  if (!isMeasuring) return;
  unsigned long now = millis();
  unsigned long elapsed = now - measureStartTime;
  unsigned long remaining = (measureDuration > elapsed) ? (measureDuration - elapsed) : 0;
  secondsLeft = remaining / 1000;
  if (micros() - lastSampleTime >= 1000000UL / SAMPLE_RATE && sampleCount < FFT_SIZE) {
    lastSampleTime = micros();
    sensors_event_t event;
    accel.getEvent(&event);
    float x = event.acceleration.x - offsetX;
    float y = event.acceleration.y - offsetY;
    float z = event.acceleration.z - offsetZ;
    float acc = sqrt(x * x + y * y + z * z);
    accBuffer[sampleCount] = acc;
    timeBuffer[sampleCount] = 0.0;
    velocitySum += acc / SAMPLE_RATE;
    if (acc > aPeak) aPeak = acc;
    sampleCount++;
  }
  if (elapsed >= measureDuration && sampleCount == FFT_SIZE) {
    isMeasuring = false;
    float sum = 0, sumSq = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
      sum += accBuffer[i];
      sumSq += accBuffer[i] * accBuffer[i];
    }
    float mean = sum / FFT_SIZE;
    aRMS = sqrt(sumSq / FFT_SIZE);
    float variance = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
      variance += pow(accBuffer[i] - mean, 2);
    }
    stdDev = sqrt(variance / FFT_SIZE);
    vRMS = velocitySum / FFT_SIZE;
    // Remoção de offset DC
    double dcOffset = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
      dcOffset += accBuffer[i];
    }
    dcOffset /= FFT_SIZE;
    for (int i = 0; i < FFT_SIZE; i++) {
      accBuffer[i] -= dcOffset;
    }
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
    double maxMag = 0;
    int index = 0;
    for (int i = 1; i < FFT_SIZE / 2; i++) {
      if (accBuffer[i] > maxMag && accBuffer[i] > 5.0) {  // Filtro de ruído
        maxMag = accBuffer[i];
        index = i;
      }
    }
    freqDominant = (index * SAMPLE_RATE) / (float)FFT_SIZE;
    Serial.print("Magnitude FFT pico: ");
    Serial.println(maxMag);
    // Filtro adicional para descartar leitura de frequência em estado estático
    if (aRMS < 0.15 && stdDev < 0.05 && vRMS < 0.01 && maxMag < 5.0) {
      freqDominant = 0;
    }
    vibroState = VibroState::VIBRO_MEASURE;
    Serial.println("\n--- RESULTADO ---");
    Serial.print("Pico Acel: ");
    Serial.print(aPeak, 3);
    Serial.println(" m/s²");
    Serial.print("RMS Acel: ");
    Serial.print(aRMS, 3);
    Serial.println(" m/s²");
    Serial.print("Desvio padrão: ");
    Serial.print(stdDev, 3);
    Serial.println(" m/s²");
    Serial.print("Velocidade RMS: ");
    Serial.print(vRMS, 3);
    Serial.println(" m/s");
    Serial.print("Freq. dominante: ");
    Serial.print(freqDominant, 2);
    Serial.println(" Hz");
    Serial.println("-----------------");
  }
}
void exibirImagemDaFlash(uint32_t enderecoInicial, int largura, int altura, int offsetX, int offsetY) {
  int bytesPorLinha = largura / 8;
  uint8_t linhaBuffer[bytesPorLinha];
  for (int y = 0; y < altura; y++) {
    // Lê a linha da imagem diretamente da flash
    uint32_t enderecoLinha = enderecoInicial + (y * bytesPorLinha);
    readData(enderecoLinha, linhaBuffer, bytesPorLinha);
    // Desenha os pixels dessa linha
    for (int byteIndex = 0; byteIndex < bytesPorLinha; byteIndex++) {
      uint8_t b = linhaBuffer[byteIndex];
      for (int bit = 0; bit < 8; bit++) {
        int pixelX = offsetX + (byteIndex * 8) + (7 - bit);  // MSB primeiro
        int pixelY = offsetY + y;
        if (pixelX >= 0 && pixelX < 128 && pixelY >= 0 && pixelY < 64) {
          bool isOn = b & (1 << bit);
          display.drawPixel(pixelX, pixelY, isOn ? WHITE : BLACK);
        }
      }
    }
  }
  display.display();
}
bool compararConfigs(const std::vector<Config> &a, const std::vector<Config> &b) {
  if (a.size() != b.size()) {
    return false;  // Tamanho diferente, então é diferente
  }
  // Compara cada elemento
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].chave != b[i].chave || a[i].valor != b[i].valor) {
      return false;  // Encontrou uma diferença
    }
  }
  return true;  // Nenhum elemento diferente foi encontrado
}
bool updateValuesRec() {
  // Compara o estado atual com o estado anterior
  if (!compararConfigs(dadosConfig, lastDadosConfig)) {
    //Serial.println("Mudanças detectadas. Salvando na EEPROM...");
    // Se houver uma diferença, salva os dados
    if (salvarDadosEEPROM(dadosConfig)) {
      // Se o salvamento for bem-sucedido, atualiza a variável do estado anterior
      lastDadosConfig = dadosConfig;
      Serial.println("Dados salvos e atualizado.");
      return true;
    } else {
      //Serial.println("Falha ao salvar. Nao atualizando lastDadosConfig.");
      return false;
    }
  }
  return false;
}
// ==== Setup dos botões ====
void setupButtons() {
  pinMode(BUTTON_MENU, INPUT_PULLUP);
  pinMode(BUTTON_SET, INPUT_PULLUP);
  pinMode(BUTTON_DOUBLE, INPUT_PULLUP);
  pinMode(BUTTON_HALF, INPUT_PULLUP);
  pinMode(BUTTON_ENC, INPUT_PULLUP);
}
void setup() {
  // Inicializa o pino do LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // Inicializa o pino do sensor IR
  pinMode(SENSOR_IR_PIN, INPUT);
  setupButtons();
  EEPROM.begin(EEPROM_SIZE);
  // Inicia a comunicação I2C com os pinos corretos
  Wire.begin(21, 22);
  // SDA = 21, SCL = 22
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Falha ao iniciar OLED"));
    while (true)
      ;
  }
  display.clearDisplay();
  adxlAvailable = accel.begin();
  if (adxlAvailable) accel.setRange(ADXL345_RANGE_4_G);
  // Inicializa o timer com resolução de 1us (1MHz)
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, onTimer);
  timerAlarm(timer, STB_partTime, true, 0);
  Serial.begin(115200);
  while (!Serial)
    ;
  iniciarSPIFlash();
  identificarJEDEC();
  exibirImagemDaFlash(0x00000, 128, 64, 0, 0);
  delay(2000);
  display.clearDisplay();
  exibirImagemDaFlash(0x0041F, 128, 32, 0, 20);
  if (EEPROM.read(0) == 0xFF) {
    Serial.println("EEPROM vazia. Gravando valores padrao...");
    // Defina seus valores padrão aqui
    setValor(dadosConfig, "IDIOMA", "1");
    setValor(dadosConfig, "TIMEMEASURE", "30");
    setValor(dadosConfig, "TIMECALIB", "10");
    setValor(dadosConfig, "FPM", "3000");
    // Salva os valores padrão na EEPROM
    salvarDadosEEPROM(dadosConfig);
  } else {
    // Se a EEPROM nao estiver em branco, carrega os dados
    carregarDadosEEPROM(dadosConfig);
    lastDadosConfig = dadosConfig;
  }
  numIdioma = getValor(dadosConfig, "IDIOMA").toInt();
  idioma = getSiglaIdioma(numIdioma);
  timeMeasure = getValor(dadosConfig, "TIMEMEASURE").toInt();
  timeCalib = getValor(dadosConfig, "TIMECALIB").toInt();
  fpm = getValor(dadosConfig, "FPM").toInt();
  delay(1000);
  carregarArquivoParcial(displayConfig, 0x006DD, 0x001FFF, idioma);
  delay(1000);
  display.clearDisplay();
}
void loop() {
  handleInput();
  updateValues();  //Stroboscópio fica com a função rodando, mas não executa os leds. Apenas os contadores do timer.
  if (adxlAvailable) {
    updateMeasurement();
    updateCalibration();
  }
  if (inMenu) {
    drawMenu();
    STB_outputEnabled = false;
    TESTE_calc = false;
    Lant_calc = false;
    vibroState = VibroState::VIBRO_HOME;
    // === Lógica do Encoder para navegação do menu principal ===
    static long lastEncoderPosition = 0;
    long newPosition = encoder.read() / 8;  // Ajuste o divisor para a sensibilidade que preferir
    if (newPosition != lastEncoderPosition) {
      int delta = newPosition - lastEncoderPosition;
      if (delta > 0) {  // Girando para a frente
        currentMode = static_cast<Mode>((static_cast<int>(currentMode) + 1) % static_cast<int>(Mode::NUM_MODES));
      } else if (delta < 0) {  // Girando para trás
        currentMode = static_cast<Mode>((static_cast<int>(currentMode) - 1 + static_cast<int>(Mode::NUM_MODES)) % static_cast<int>(Mode::NUM_MODES));
      }
      lastEncoderPosition = newPosition;
    }
  } else {
    drawScreen(currentMode);
  }
  if (!inMenu) {
    switch (selectedMode) {
      case Mode::FREQUENCY:
        {
          STB_outputEnabled = true;
          //Comandos para alterar o valor usando o Encoder
          long newPos = encoder.read() / 8;
          // Dividido por 8 para reduzir sensibilidade
          if (inSubmenu) {
            int delta = newPos - STB_lastEncoderPos;
            STB_phaseDegrees = constrain(STB_phaseDegrees + delta, 0, 359);
            STB_lastEncoderPos = newPos;
            saveData = setValor(dadosConfig, "FPM", String(fpm));
            STB_calc = true;
          } else if (!inSubmenu) {
            int delta = newPos - STB_lastEncoderPos;
            if (inEncoder) {
              delta = delta * 10;
            }
            fpm = constrain(fpm + delta, minFPM, maxFPM);
            STB_lastEncoderPos = newPos;
            saveData = setValor(dadosConfig, "FPM", String(fpm));
            STB_calc = true;
          }
          break;
        }
      case Mode::RPM:
        {
          unsigned long agora = millis();
          bool estadoAtual = digitalRead(SENSOR_IR_PIN);
          // Detecta pulso (borda de subida: LOW -> HIGH)
          if (estadoAtual == HIGH && estadoAnterior == LOW) {
            contagemPulsos++;
          }
          estadoAnterior = estadoAtual;
          // A cada TEMPO_LEITURA_RPM milissegundos, calcula e imprime o RPM
          if (agora - ultimaLeitura >= TEMPO_LEITURA_RPM) {
            ultimaLeitura = agora;
            // Se há 1 marca por rotação, então: RPM = pulsos * 60
            int rpm = contagemPulsos * 60;
            rpmValue = rpm;
            // Reinicia a contagem
            contagemPulsos = 0;
          }
          break;
        }
      case Mode::LANTERN:
        STB_outputEnabled = true;
        Lant_calc = true;
        break;
      case Mode::TEST:
        {
          static unsigned long ultimoTempo = 0;
          static float fpmAtual = 30.0;
          FreqDeTest = fpmAtual/60;
          static const float passo = 30.0;
          static const unsigned long intervaloTeste = 100;  // 2 segundos entre testes
          unsigned long agora = millis();
          if (fpmTest.isRunning()) {
            if (agora - ultimoTempo >= intervaloTeste) {
              TESTE_fpm = fpmAtual;
              updateValues();
              fpmAtual += passo;
              FreqDeTest = fpmAtual/60;
              if (fpmAtual >= maxFPM) {
                fpmAtual = 30.0;  // reinicia o ciclo
              }
              ultimoTempo = agora;
            }
          } else if (fpmTest.isExpired()) {
            fpmAtual = 30;
            FreqDeTest = fpmAtual/60;
            STB_outputEnabled = false;
            TESTE_calc = false;
          } else {
            STB_outputEnabled = false;
            TESTE_calc = false;
          }
          break;
        }
      case Mode::HOME:
        {
          static long lastEncoderPos = 0;
          long newPos = encoder.read() / 8;
          if (newPos != lastEncoderPos) {
            int delta = newPos - lastEncoderPos;
            if (delta > 0) {  // Girando para a frente
              if (topLineIndex < totalLines - lineShowMsg) {
                topLineIndex++;
              }
            } else if (delta < 0) {  // Girando para trás
              if (topLineIndex > 0) {
                topLineIndex--;
              }
            }
            lastEncoderPos = newPos;
          }
          break;
        }
    }
  }
  // nada aqui
}
void handleInput() {
  static bool lastMenuState = HIGH;
  static unsigned long lastDebounceTimeMenu = 0;
  static bool lastSetState = HIGH;
  static unsigned long lastDebounceTimeSet = 0;
  static bool lastDoubleState = HIGH;
  static unsigned long lastDebounceTimeDouble = 0;
  static bool lastHalfState = HIGH;
  static unsigned long lastDebounceTimeHalf = 0;
  static bool lastEncState = HIGH;
  static unsigned long lastDebounceTimeEnc = 0;
  if (checkButtonDebounce(BUTTON_MENU, lastMenuState, lastDebounceTimeMenu, 50000)) {
    if (inSubmenu) {
      // Se estiver no submenu, apenas sai dele
      inSubmenu = false;
    } else if (!inMenu) {
      // Se estiver fora do menu, entrar no menu com o modo atual
      currentMode = selectedMode;
      inMenu = true;
      updateValuesRec();
    } else {
      //currentMode = static_cast<Mode>((static_cast<int>(currentMode) + 1) % static_cast<int>(Mode::NUM_MODES));
    }
  }
  if (checkButtonDebounce(BUTTON_SET, lastSetState, lastDebounceTimeSet, 50000)) {
    if (inMenu) {
      selectedMode = currentMode;
      inMenu = false;
    } else {
      if (selectedMode == Mode::VIBROMETER) {
        // === Lógica do Encoder para ajustar o tempo ===
        static long lastEncoderPos = 0;
        long newPos = encoder.read() / 8;  // Lê a posição e reduz a sensibilidade
        if (newPos != lastEncoderPos) {
          int delta = newPos - lastEncoderPos;
          switch (vibroState) {
            case VibroState::VIBRO_CALIB:
              // Ajusta o tempo de calibração em incrementos de 5 segundos
              timeCalib = constrain(timeCalib + (delta * 5), 5, 30);
              saveData = setValor(dadosConfig, "TIMECALIB", String(timeCalib));
              break;
            case VibroState::VIBRO_IDLE:
              // Ajusta o tempo de medição em incrementos de 10 segundos
              timeMeasure = constrain(timeMeasure + (delta * 10), 10, 60);
              saveData = setValor(dadosConfig, "TIMEMEASURE", String(timeMeasure));
              break;
            default:
              // Não faz nada em outros estados do vibrometro
              break;
          }
          lastEncoderPos = newPos;
        }
        switch (vibroState) {
          //Clique SET para iniciar
          case VibroState::VIBRO_HOME:
            vibroState = VibroState::VIBRO_CALIB;
            break;
          case VibroState::VIBRO_CONFIG:
            //vibroState = VibroState::VIBRO_MEASURE;
            //measureSeismicActivity();
            break;
          case VibroState::VIBRO_MEASURE:
            vibroState = VibroState::VIBRO_RESULT;
            break;
          case VibroState::VIBRO_RESULT:
            vibroState = VibroState::VIBRO_HOME;
            break;
          default:
            // Não faz nada em outros estados do vibrometro
            break;
        }
      } else if (selectedMode == Mode::RPM) {
        inSubmenu = false;
        if (rpmValue >= 30) {
          fpm = rpmValue;
          //FPM recebe o valor de RPM
          setValor(dadosConfig, "FPM", String(fpm));
          msgTimer.startTimer(2);  //Inicia a contagem para exibir a mensagem de gravando por 2 segundos
        }
      } else if (selectedMode == Mode::TEST) {
        STB_outputEnabled = true;
        TESTE_calc = true;
        fpmTest.startTimer(15);
      } else if (selectedMode == Mode::LANTERN || currentMode == Mode::ABOUT) {
        inSubmenu = false;
      } else {
        inSubmenu = !inSubmenu;
        STB_outputEnabled = false;
      }
    }
  }
  if (checkButtonDebounce(BUTTON_DOUBLE, lastDoubleState, lastDebounceTimeDouble, 50000)) {
    if (inMenu) {
      currentMode = static_cast<Mode>((static_cast<int>(currentMode) + 1) % static_cast<int>(Mode::NUM_MODES));
    } else if (selectedMode == Mode::HOME) {
      if (topLineIndex < totalLines - lineShowMsg) {  // lineShowMsg = linhas cabem na tela
        topLineIndex++;
      }
    } else if (currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_CALIB) {
      timeCalib = constrain(timeCalib - 5, 5, 30);
    } else if (currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_IDLE) {
      timeMeasure = constrain(timeMeasure - 10, 10, 60);
    } else if (currentMode == Mode::CONFIG) {
      numIdioma = (numIdioma - 1 + 1) % 4 + 1;
      setValor(dadosConfig, "IDIOMA", String(numIdioma));
    } else {
      if (selectedMode == Mode::FREQUENCY && !inSubmenu) {
        adjustFPM(0.5);
        STB_calc = true;
      }
    }
  }
  if (checkButtonDebounce(BUTTON_HALF, lastHalfState, lastDebounceTimeHalf, 50000)) {
    if (inMenu) {
      currentMode = static_cast<Mode>((static_cast<int>(currentMode) - 1 + static_cast<int>(Mode::NUM_MODES)) % static_cast<int>(Mode::NUM_MODES));
    } else if (selectedMode == Mode::HOME) {
      if (topLineIndex > 0) {
        topLineIndex--;
      }
    } else if (currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_CALIB) {
      timeCalib = constrain(timeCalib + 5, 5, 30);
    } else if (currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_IDLE) {
      timeMeasure = constrain(timeMeasure + 10, 10, 60);
    } else if (currentMode == Mode::CONFIG) {
      numIdioma = (numIdioma - 1 - 1 + 4) % 4 + 1;
      setValor(dadosConfig, "IDIOMA", String(numIdioma));
    } else {
      if (selectedMode == Mode::FREQUENCY && !inSubmenu) {
        adjustFPM(2.0);
        STB_calc = true;
      }
    }
  }
  if (checkButtonDebounce(BUTTON_ENC, lastEncState, lastDebounceTimeEnc, 50000)) {
    inEncoder = !inEncoder;
    if (inEncoder) {
      //FAZ
    } else {
      //NÃO FAZ
    }
  }
}
// ==== Tela do Menu ====
void drawMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(getValor(displayConfig, "SELECTMODE", idioma));
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println(getModeName(currentMode));
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.println(getValor(displayConfig, "MENUPROX", idioma));
  display.display();
}
// ==== Tela de cada Modo ====
void drawScreen(Mode mode) {
  // Limita a taxa de atualização para evitar flickering e consumo de CPU
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 50) return;
  // Atualiza a cada 50ms (20 FPS)
  lastUpdate = millis();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(getModeName(mode));
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.setCursor(0, 14);
  if (currentMode == Mode::HOME) {
    display.println("Agradecimentos: ");
    for (int i = 0; i < lineShowMsg; i++) {
      int lineIndex = topLineIndex + i;
      if (lineIndex < totalLines) {
        display.println(lines[lineIndex]);
      }
    }
  } else if (currentMode == Mode::RPM) {
    display.print(getValor(displayConfig, "RPM", idioma) + " ");
    display.print((int)rpmValue);
    display.setCursor(0, 26);
    display.print(msgTimer.isRunning() ? getValor(displayConfig, "RECRPM", idioma) : getValor(displayConfig, "SETRECRPM", idioma));
    display.setCursor(0, 56);
    display.println(getValor(displayConfig, "MENUPROX", idioma));
  } else if (currentMode == Mode::ABOUT) {
    display.println("Grupo AlfaS: ");
    display.println(getValor(displayConfig, "ABOUTP1", idioma));
    display.println(getValor(displayConfig, "ABOUTP2", idioma));
    display.println(getValor(displayConfig, "ABOUTP3", idioma));
    display.println(getValor(displayConfig, "ABOUTP4", idioma));
    display.println(getValor(displayConfig, "ABOUTSITE", idioma));
    exibirImagemDaFlash(0x0041F, 32, 32, 90, 15);
  } else if (currentMode == Mode::TEST) {
    display.print(fpmTest.isRunning() ? getValor(displayConfig, "TESTING", idioma) : getValor(displayConfig, "TESTER", idioma));
    display.setCursor(45, 19);
    display.println(FreqDeTest);
    display.setCursor(0, 56);
    display.println(fpmTest.isRunning() ? getValor(displayConfig, "TESTWAIT", idioma) : getValor(displayConfig, "TESTSET", idioma));
  } else if (currentMode == Mode::LANTERN) {
    display.print(getValor(displayConfig, "LANTERN", idioma));
    display.setCursor(0, 56);
    display.println(getValor(displayConfig, "MENULANT", idioma));
  } else if (currentMode == Mode::CONFIG) {
    display.println(getValor(displayConfig, "SELECTLANG", idioma));
    display.println(getNomeIdioma(numIdioma));
  } else if (currentMode == Mode::FREQUENCY) {
    display.setTextSize(2);
    display.setCursor(0, 14);
    display.print((int)fpm);
    display.setTextSize(1);
    display.setCursor(70, 22);
    display.print(getValor(displayConfig, "FPM", idioma));
    display.setCursor(0, 34);
    display.print(getValor(displayConfig, "HZ", idioma) + "");
    display.print(fpm / 60.0, 2);
    display.setCursor(0, 46);
    display.print(getValor(displayConfig, "PHASE", idioma) + " ");
    display.print(STB_phaseDegrees);
    display.print((char)247);
    if (inSubmenu) {
      display.setCursor(80, 46);
      display.println("<");
      display.setCursor(0, 56);
      display.println(getValor(displayConfig, "PHASEEDIT", idioma));
    }
  } else if (currentMode == Mode::VIBROMETER) {
    if (adxlAvailable) {
      switch (vibroState) {
        case VibroState::VIBRO_HOME:
          display.println(getValor(displayConfig, "CALIBRATE", idioma));
          break;
        case VibroState::VIBRO_CALIB:
          if (!isCalibrating) {
            display.print(getValor(displayConfig, "SELECTTIME", idioma));
            display.print(timeCalib);
            display.println("s");
            display.println("");
            display.println(getValor(displayConfig, "SETCALIB", idioma));
          } else {
            display.println(getValor(displayConfig, "CALIBRATING", idioma));
            display.println("");
            display.print(getValor(displayConfig, "RESTTIME", idioma));
            display.print(secondsLeftCalib);
            display.println("s");
          }
          break;
        case VibroState::VIBRO_IDLE:
          display.print(getValor(displayConfig, "SELECTTIME", idioma));
          display.print(timeMeasure);
          display.println("s");
          display.println("");
          display.println(getValor(displayConfig, "MSGIDLE1", idioma));
          display.println(getValor(displayConfig, "MSGIDLE2", idioma));
          break;
        case VibroState::VIBRO_CONFIG:
          display.println(getValor(displayConfig, "MEASURE", idioma));
          display.println("");
          display.print(getValor(displayConfig, "RESTTIME", idioma));
          display.print(secondsLeft);
          display.println("s");
          break;
        case VibroState::VIBRO_MEASURE:
          display.print("PA: ");
          display.print(aPeak, 3);
          display.println("m/s2");
          display.print("RMS A: ");
          display.print(aRMS, 3);
          display.println("m/s2");
          display.print("DP: ");
          display.print(stdDev, 3);
          display.println("m/s2");
          display.print("V RMS: ");
          display.print(vRMS, 3);
          display.println("m/s");
          display.print("FD: ");
          display.print(freqDominant, 2);
          display.println("Hz");
          break;
        case VibroState::VIBRO_RESULT:
          display.println(getValor(displayConfig, "RESULTPA", idioma));
          display.println(getValor(displayConfig, "RESULTRMSA", idioma));
          display.println(getValor(displayConfig, "RESULTDP", idioma));
          display.println(getValor(displayConfig, "RESULTVRMS", idioma));
          display.println(getValor(displayConfig, "RESULTFD", idioma));
          break;
      }
    } else {
      display.println(getValor(displayConfig, "NOTSENSOR", idioma));
    }
  }
  display.display();
}
// ==== Retorna nome do modo atual ====
String getModeName(Mode mode) {
  switch (mode) {
    case Mode::HOME: return "StroboTech";
    case Mode::FREQUENCY: return getValor(displayConfig, "TITLEFREQ", idioma);
    case Mode::RPM: return getValor(displayConfig, "TITLERPM", idioma);
    case Mode::LANTERN: return getValor(displayConfig, "TITLELANT", idioma);
    case Mode::VIBROMETER: return getValor(displayConfig, "TITLEVIBRO", idioma);
    case Mode::TEST: return getValor(displayConfig, "TITLETEST", idioma);
    case Mode::ABOUT: return getValor(displayConfig, "TITLEABOUT", idioma);
    case Mode::CONFIG: return getValor(displayConfig, "TITLECONFIG", idioma);
  }
}
String getNomeIdioma(int lang) {
  switch (lang) {
    case 1: return getValor(displayConfig, "LANGPT", idioma);
    case 2: return getValor(displayConfig, "LANGEN", idioma);
    case 3: return getValor(displayConfig, "LANGES", idioma);
    case 4: return getValor(displayConfig, "LANGFR", idioma);
  }
}
String getSiglaIdioma(int lng) {
  switch (lng) {
    case 1: return "PT";
    case 2: return "EN";
    case 3: return "ES";
    case 4: return "FR";
  }
}
