#include "flash.h"
#include "config.h"
#include "spiffsConfig.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_LSM6DS.h>
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
#include <arduinoFFT.h>
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "VibroRMS.h"

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

WebServer server(80);
bool otaAtivo = false;
#define GERAHTML 0

// ==== Pinos ====
#define ENCODER_PIN_A 33
#define ENCODER_PIN_B 32
#define BUTTON_MENU 25
#define BUTTON_SET 13
#define BUTTON_HALF 27
#define BUTTON_DOUBLE 26
#define BUTTON_ENC 17
#define ONOFF 14
#define LED_PIN 2        // Pino do LED de alta potência para o estroboscópio/lanterna
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
int Sensib_Encoder = 8;
//Acelerometros
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(123);
bool adxlAvailable = false;
Adafruit_LSM6DS lsm6ds;
bool lsmAvailable = false;
//Fim Acelerometros

bool AcelAvailable = false;

// ==== Menu ====
  enum class Mode { HOME, FREQUENCY, RPM, LANTERN, VIBROMETER, TEST, ABOUT, CONFIG, NUM_MODES };
  Mode currentMode = Mode::HOME;
  Mode selectedMode = Mode::HOME;
  bool inMenu = true;
  bool inSubmenu = false;
  bool inEncoder = false;
  int valInEncoder = 0;
  int modeFreq = 0;
  int modeConfig = 0;
  int modeCalibFatRpm = 0;
  int modeMeasure = 0;
// ==== FIM Menu ====

// ==== Variáveis do Vibrometro ====
  enum class VibroState { VIBRO_HOME, VIBRO_SELECTCALIB, VIBRO_CALIBRATING, VIBRO_SELECTMEASURE, VIBRO_MEASURING, VIBRO_MEASURERESULT, VIBRO_ABOUT };
  VibroState vibroState = VibroState::VIBRO_HOME;
  // ==== Medição ====
  float ax, ay, az;
  //-------------------------------------
  // ==== Variáveis de Gravação do tempo de vibração ====
  int timeCalib = 15;
// ==== FIM Variáveis do Vibrometro ====

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
    "Renato L. Cruz",
    "Vitor Santarosa",
    "Alex Penteado",
    "Bruno de Campos",
    "Fernando A. Bosco",
    "Fabio Camarinha",
    "Gabriela Viana"
  };
  const int totalLines = sizeof(lines) / sizeof(lines[0]);
  int topLineIndex = 0;
  // Índice da linha superior visível
  int lineShowMsg = 6;
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
int indexFile = 1;
// ==============
//Encoder
long lastEncoderPos = 0;
// ==== TESTE ====
bool lanterna = false;
bool TESTE_calc = false;
float TESTE_fpm = 0;

// ==== RPM ====
  int rpmValue = 0;
  float CalibFat = 0.555;    // fator inicial
  float step = 0.000;        // passo de variação
  float minVal = 0.000;      // limite mínimo
  float maxVal = 4.000;      // limite máximo

  volatile int count = 0;
  unsigned long lastMillis = 0;
  int divisor = 1;

  void loadRPM(){
    unsigned long currentMillis = millis();
    if (currentMillis - lastMillis >= 1000) {
      lastMillis = currentMillis;
      
      rpmValue = (count * 60) / divisor*CalibFat;
      
      count = 0;
    }
  }

  void countWhiteStripe() {
    count++;
  }

// ==== FIM RPM ====

// ==== STROBOSCOPIO ==== //
  // ==== Variáveis do Modo Estroboscópio ====
  int fpm = 683;
  const int minFPM = 30;     // Limite mínimo de FPM
  const int maxFPM = 35000;  // Limite máximo de FPM

  int dutyCycle = 4;
  const int minDuty = 1;
  const int maxDuty = 100;


  // ==== Variáveis do Modo Estroboscópio ====
  long STB_lastEncoderPos = 0;
  int STB_phaseDegrees = 0;
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

  volatile unsigned long STB_timeHigh = 0;
  volatile unsigned long STB_timeLow = 0;
  // ==== Função de interrupção do timer. Alterna o estado do LED e aplica o atraso de fase na primeira chamada. ====
  void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&STB_timerMux);

    if (!STB_outputEnabled) {
      GPIO.out_w1tc = (1 << LED_PIN);  // Força LOW
      portEXIT_CRITICAL_ISR(&STB_timerMux);
      return;
    }

    // Aplica fase apenas no primeiro pulso
    if (STB_firstPulse && STB_phaseDelayMicros > 0) {
      timerAlarm(timer, STB_phaseDelayMicros, true, 0);
      STB_firstPulse = false;
      portEXIT_CRITICAL_ISR(&STB_timerMux);
      return;
    }

    // Alterna o LED
    STB_pulseState = !STB_pulseState;
    if (STB_pulseState) {
      GPIO.out_w1ts = (1 << LED_PIN);  // HIGH
    } else {
      GPIO.out_w1tc = (1 << LED_PIN);  // LOW
    }

    // Define próximo intervalo: HIGH ou LOW
    unsigned long nextInterval = STB_pulseState ? STB_timeHigh : STB_timeLow;
    timerAlarm(timer, nextInterval, true, 0);

    portEXIT_CRITICAL_ISR(&STB_timerMux);
  }
  // Atualiza os valores com base na entrada de FPM. Recalcula os tempos de ciclo e atraso de fase.
  void updateValues() {
    // Tempo total de ciclo em microssegundos (um ciclo completo do FPM)
    unsigned long cycleTimeMicros = 60000000UL / fpm;
    if (TESTE_calc) {
      cycleTimeMicros = 60000000UL / TESTE_fpm;
    }
    // Duty mínimo de 1% para flashes curtos
    float dutyFraction = (dutyCycle < 1) ? 0.01f : ((float)dutyCycle / 100.0f);

    // Calcula tempo de HIGH e LOW do LED
    STB_timeHigh = cycleTimeMicros * dutyFraction;
    STB_timeLow = cycleTimeMicros - STB_timeHigh;

    // Calcula o atraso de fase
    STB_phaseDelayMicros = (STB_phaseDegrees / 360.0f) * cycleTimeMicros;

    // Prepara para o primeiro pulso
    STB_firstPulse = true;
    STB_calc = false;
  }
// Fim Stroboscópio

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
  // --- VARIÁVEIS GLOBAIS ---
  //logCsvSample((timeBuffer[sampleCount]*10)/3, acc, x, y, z);

  bool isCalibrating = false;
  bool isMeasuring = false;
  float SAMPLE_RATE = 200.0f;
  #define MAX_FFT_SIZE 2048
  int FFT_SIZE = 512;     //Resolução da frequência 256, 512 ou 1024

  unsigned long calibrationStartTime = 0;
  unsigned long calibrationDuration = 0;
  unsigned long lastSampleTime = 0;

  int samplesRemaining = 0;

  float offsetX = 0;
  float offsetY = 0;
  float offsetZ = 0;

  float accBuffer[MAX_FFT_SIZE];
  float imagBuffer[MAX_FFT_SIZE];
  float timeBuffer[MAX_FFT_SIZE];
  float velocityXBuffer[MAX_FFT_SIZE];
  float velocityYBuffer[MAX_FFT_SIZE];
  float velocityZBuffer[MAX_FFT_SIZE];

  float aPeak = 0;
  float aRMS = 0;
  float vRMS = 0;
  float stdDev = 0;
  float freqDominant = 0;

  int sampleCount = 0;
  int secondsLeftCalib = 0;
  int secondsLeft = 0;

  // FFT
  ArduinoFFT<float> FFT = ArduinoFFT<float>();

  // --- CALIBRAÇÃO ---
  void startCalibration(int durationSeconds) {
      isCalibrating = true;
      calibrationDuration = durationSeconds * 1000UL;
      calibrationStartTime = millis();
      offsetX = 0;
      offsetY = 0;
      offsetZ = 0;
      sampleCount = 0;
      secondsLeftCalib = durationSeconds;
      Serial.println("Iniciando calibração...");
  }

  void updateCalibration() {
      if (!isCalibrating) return;

      unsigned long elapsed = millis() - calibrationStartTime;
      secondsLeftCalib = (calibrationDuration - elapsed) / 1000;

      float x, y, z;

      if (adxlAvailable) {
          sensors_event_t event;
          accel.getEvent(&event);
          x = event.acceleration.x;
          y = event.acceleration.y;
          z = event.acceleration.z;
      } else {
          sensors_event_t accelEvent, gyroEvent, tempEvent;
          lsm6ds.getEvent(&accelEvent, &gyroEvent, &tempEvent);
          x = accelEvent.acceleration.x;
          y = accelEvent.acceleration.y;
          z = accelEvent.acceleration.z;
      }

      offsetX += x;
      offsetY += y;
      offsetZ += z;
      sampleCount++;

      if (elapsed >= calibrationDuration) {
          offsetX /= sampleCount;
          offsetY /= sampleCount;
          offsetZ /= sampleCount;

          isCalibrating = false;
          vibroState = VibroState::VIBRO_SELECTMEASURE;

          Serial.println("Calibração concluída!");
          Serial.printf("Offset X: %.3f\nOffset Y: %.3f\nOffset Z: %.3f\n", offsetX, offsetY, offsetZ);
      }
  }

 // =====================
  // Inicia medição
  // =====================
  VibroRMS rmsX, rmsY, rmsZ;

  void startMeasurement() {
    isMeasuring = true;
    sampleCount = 0;

    aPeak = 0;
    aRMS = 0;
    stdDev = 0;
    vRMS = 0;
    freqDominant = 0;
    lastSampleTime = 0;

    // Inicializa medidores RMS
    rmsX.reset();
    rmsY.reset();
    rmsZ.reset();

    Serial.printf("Iniciando medição... (%d amostras @ %.1f Hz)\n", FFT_SIZE, SAMPLE_RATE);
    Serial.printf("Duração estimada: %.3f s\n", FFT_SIZE / SAMPLE_RATE);

    // Inicializa CSV
    if (!SPIFFS.begin(true)) {
        Serial.println("❌ Erro ao montar SPIFFS");
        return;
    }
    startCsvLog(indexFile);
}

  void updateMeasurement() {
    if (!isMeasuring) return;

    static float lastAcc[3] = {NAN, NAN, NAN};
    static float lastX[3] = {NAN, NAN, NAN};
    static float lastY[3] = {NAN, NAN, NAN};
    static float lastZ[3] = {NAN, NAN, NAN};
    static int repeatIndex = 0;

    // Calcula quantas amostras faltam
    int samplesLeft = FFT_SIZE - sampleCount;
    if (samplesLeft < 0) samplesLeft = 0;

    // Armazena globalmente (caso queira usar em outro lugar)
    samplesRemaining = samplesLeft;

    // Captura amostra conforme taxa de amostragem
    if (micros() - lastSampleTime >= 1000000UL / SAMPLE_RATE && sampleCount < FFT_SIZE) {
        lastSampleTime = micros();

        float x = 0, y = 0, z = 0, acc = 0;

        // --- Leitura do sensor ---
        if (adxlAvailable) {
            sensors_event_t event;
            accel.getEvent(&event);
            x = event.acceleration.x - offsetX;
            y = event.acceleration.y - offsetY;
            z = event.acceleration.z - offsetZ;
        } else {
            sensors_event_t accelEvent, gyroEvent, tempEvent;
            lsm6ds.getEvent(&accelEvent, &gyroEvent, &tempEvent);
            x = accelEvent.acceleration.x - offsetX;
            y = accelEvent.acceleration.y - offsetY;
            z = accelEvent.acceleration.z - offsetZ;
        }

        acc = sqrt(x * x + y * y + z * z);

        // --- RMS incremental (EmonLib) ---
        rmsX.addSample(x);
        rmsY.addSample(y);
        rmsZ.addSample(z);

        // --- Verifica repetição de amostras ---
        bool repeat = false;
        if (!isnan(lastAcc[0]) && !isnan(lastAcc[1]) && !isnan(lastAcc[2])) {
            if (acc == lastAcc[0] && acc == lastAcc[1] && acc == lastAcc[2] &&
                x == lastX[0] && x == lastX[1] && x == lastX[2] &&
                y == lastY[0] && y == lastY[1] && y == lastY[2] &&
                z == lastZ[0] && z == lastZ[1] && z == lastZ[2]) {
                repeat = true;
            }
        }

        if (!repeat) {
            accBuffer[sampleCount] = acc;
            imagBuffer[sampleCount] = 0.0;
            timeBuffer[sampleCount] = (float)sampleCount / SAMPLE_RATE;

            float dt = 1.0 / SAMPLE_RATE;
            if (sampleCount == 0) {
                velocityXBuffer[0] = 0;
                velocityYBuffer[0] = 0;
                velocityZBuffer[0] = 0;
            } else {
                velocityXBuffer[sampleCount] = velocityXBuffer[sampleCount - 1] + x * dt;
                velocityYBuffer[sampleCount] = velocityYBuffer[sampleCount - 1] + y * dt;
                velocityZBuffer[sampleCount] = velocityZBuffer[sampleCount - 1] + z * dt;
            }

            if (fabs(acc) > aPeak) aPeak = fabs(acc);

            // Salva no CSV
            logCsvSample(SAMPLE_RATE,timeBuffer[sampleCount], acc, x, y, z);

            sampleCount++;
        }

        // Atualiza buffers de repetição
        lastAcc[repeatIndex] = acc;
        lastX[repeatIndex] = x;
        lastY[repeatIndex] = y;
        lastZ[repeatIndex] = z;
        repeatIndex = (repeatIndex + 1) % 3;
    }

    // Finaliza medição quando atingido FFT_SIZE
    if (sampleCount >= FFT_SIZE) {
        isMeasuring = false;
        csvFile.flush();

        if (endCsvLog()) {
            indexFile++;
            setValor(dadosConfig, "INDEXFILE", String(indexFile));
            updateValuesRec();
        }

        // --- Remove DC antes da FFT ---
        float mean = 0;
        for (int i = 0; i < sampleCount; i++) mean += accBuffer[i];
        mean /= sampleCount;
        for (int i = 0; i < sampleCount; i++) accBuffer[i] -= mean;

        // --- RMS usando EmonLib ---
        float rmsXval = rmsX.getRMS();
        float rmsYval = rmsY.getRMS();
        float rmsZval = rmsZ.getRMS();
        aRMS = sqrt(rmsXval * rmsXval + rmsYval * rmsYval + rmsZval * rmsZval);

        rmsX.reset();
        rmsY.reset();
        rmsZ.reset();

        // --- Desvio padrão ---
        float variance = 0;
        for (int i = 0; i < sampleCount; i++)
            variance += accBuffer[i] * accBuffer[i];
        stdDev = sqrt(variance / sampleCount);

        // --- Velocidade RMS ---
        float sumVelSq = 0;
        for (int i = 0; i < sampleCount; i++) {
            float vMag = sqrt(
                velocityXBuffer[i] * velocityXBuffer[i] +
                velocityYBuffer[i] * velocityYBuffer[i] +
                velocityZBuffer[i] * velocityZBuffer[i]
            );
            sumVelSq += vMag * vMag;
        }
        vRMS = sqrt(sumVelSq / sampleCount);

        // --- FFT ---
        FFT.windowing(accBuffer, sampleCount, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(accBuffer, imagBuffer, sampleCount, FFT_FORWARD);
        FFT.complexToMagnitude(accBuffer, imagBuffer, sampleCount);

        // --- Freq. dominante ---
        float maxMag = 0;
        int index = 0;
        for (int i = 1; i < sampleCount / 2; i++) {
            if (accBuffer[i] > maxMag) {
                maxMag = accBuffer[i];
                index = i;
            }
        }
        freqDominant = (index * SAMPLE_RATE) / (float)sampleCount;

        vibroState = VibroState::VIBRO_MEASURERESULT;

        // --- Resultado ---
        Serial.println("\n--- RESULTADO ---");
        Serial.printf("Amostras coletadas: %d\n", sampleCount);
        Serial.printf("Pico Acel: %.3f m/s²\n", aPeak);
        Serial.printf("RMS Acel: %.3f m/s²\n", aRMS);
        Serial.printf("Desvio padrão: %.3f m/s²\n", stdDev);
        Serial.printf("Velocidade RMS: %.3f m/s\n", vRMS);
        Serial.printf("Freq. dominante: %.2f Hz\n", freqDominant);
        Serial.println("-----------------");
    }
}



// ==== FIM Funçoes do Vibrometro calibrar e medir ====

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
  pinMode(BUTTON_HALF, INPUT_PULLUP);
  pinMode(BUTTON_DOUBLE, INPUT_PULLUP);
  pinMode(BUTTON_ENC, INPUT_PULLUP);
}
//OTA
  const char* ssidOTA = "StroboTech";
  const char* senhaOTA = "12345678";

#ifdef GERAHTML
  const char* htmlContent = R"rawliteral(
    <!DOCTYPE html>
    <html lang="pt">
    <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Gerenciamento Strobotech</title>
    <style>
    body {
      font-family: Arial, sans-serif;
      background-color: #f0f0f0;
      text-align: center;
      margin: 0;
      padding: 0;
    }
    .container {
      max-width: 500px;
      background: #fff;
      padding: 20px;
      margin: 30px auto;
      border-radius: 10px;
      box-shadow: 0px 4px 10px rgba(0,0,0,0.1);
    }
    h2 {
      color: #333;
    }
    input[type='file'] {
      margin: 15px 0;
      width: 100%;
    }
    input[type='submit'], button {
      background-color: #4CAF50;
      color: white;
      border: none;
      padding: 10px 20px;
      font-size: 16px;
      border-radius: 5px;
      cursor: pointer;
      margin-top: 10px;
    }
    input[type='submit']:hover, button:hover {
      background-color: #45a049;
    }
    .progress {
      width: 100%;
      background-color: #ddd;
      border-radius: 5px;
      margin-top: 10px;
    }
    .progress-bar {
      width: 0%;
      height: 20px;
      background-color: #4CAF50;
      border-radius: 5px;
      text-align: center;
      color: white;
      transition: width 0.3s;
    }
    #fileList {
      margin-top: 20px;
    }
    .file-item {
      background: #fafafa;
      margin: 5px 0;
      padding: 10px;
      border-radius: 5px;
      box-shadow: 0px 1px 3px rgba(0,0,0,0.1);
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .file-item button {
      background-color: #2196F3;
      font-size: 14px;
      padding: 5px 10px;
    }
    .file-item button.delete {
      background-color: #f44336;
    }
    .file-item button:hover {
      opacity: 0.9;
    }
    p {
      color: #555;
      font-size: 14px;
      margin: 10px 0;
    }
    </style>
    </head>
    <body>

    <div class="container">
      <h2>Atualização OTA do Firmware</h2>
      <p>Escolha o arquivo .bin do firmware para atualizar seu dispositivo</p>
      <form id="uploadForm">
        <input type="file" name="update" id="updateFile" accept=".bin" required><br>
        <input type="submit" value="Atualizar Firmware">
        <div class="progress"><div class="progress-bar" id="progressBar">0%</div></div>
      </form>
      <p>Atenção: Não desligue o dispositivo durante a atualização!</p>
    </div>

    <div class="container">
      <h2>Arquivos CSV do Vibrometro</h2>
      <div id="fileList">Carregando...</div>
    </div>

    <script>
    const form = document.getElementById('uploadForm');
    const progressBar = document.getElementById('progressBar');
    const fileList = document.getElementById('fileList');

    // ---- Upload OTA ----
    form.addEventListener('submit', function(e) {
      e.preventDefault();
      const file = document.getElementById('updateFile').files[0];
      if (!file) return alert('Selecione um arquivo .bin');
      const xhr = new XMLHttpRequest();
      const formData = new FormData();
      formData.append('update', file);
      xhr.open('POST', '/update', true);
      xhr.upload.onprogress = function(event) {
        if (event.lengthComputable) {
          const percent = Math.round((event.loaded / event.total) * 100);
          progressBar.style.width = percent + '%';
          progressBar.textContent = percent + '%';
        }
      };
      xhr.onload = function() {
        if (xhr.status === 200) {
          progressBar.style.width = '100%';
          progressBar.textContent = 'Upload concluído!';
          alert('Atualização enviada! O dispositivo reiniciará se a OTA for bem-sucedida.');
        } else {
          alert('Erro na atualização!');
        }
      };
      xhr.send(formData);
    });

    // ---- Listar arquivos CSV ----
    function loadFiles() {
      fetch('/list')
        .then(response => response.json())
        .then(files => {
          if (files.length === 0) {
            fileList.innerHTML = "<p>Nenhum arquivo CSV encontrado.</p>";
            return;
          }
          fileList.innerHTML = "";
          files.forEach(file => {
            const div = document.createElement('div');
            div.classList.add('file-item');
            div.innerHTML = `
              <span>${file.name} (${file.size} bytes)</span>
              <div>
                <button onclick="downloadFile('${file.name}')">Baixar</button>
                <button class="delete" onclick="deleteFile('${file.name}')">Excluir</button>
              </div>
            `;
            fileList.appendChild(div);
          });
        })
        .catch(() => {
          fileList.innerHTML = "<p>Erro ao carregar lista de arquivos.</p>";
        });
    }

    // ---- Download de arquivo ----
    function downloadFile(filename) {
      window.location.href = '/download?file=' + encodeURIComponent(filename);
    }

    // ---- Excluir arquivo ----
    function deleteFile(filename) {
      if (!confirm('Deseja realmente excluir ' + filename + '?')) return;
      fetch('/delete?file=' + encodeURIComponent(filename))
        .then(response => response.text())
        .then(msg => {
          alert(msg);
          loadFiles();
        })
        .catch(() => alert('Erro ao excluir arquivo.'));
    }

    // ---- Carrega lista ao abrir ----
    window.onload = loadFiles;
    </script>

    </body>
    </html>
    )rawliteral";

#endif

  void iniciarOTA_AP() {
    if (otaAtivo) return;  // já ativo
    otaAtivo = true;

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssidOTA, senhaOTA);
    IPAddress ip = WiFi.softAPIP();
    Serial.print("OTA ativo em ");
    Serial.println(ip);

    server.serveStatic("/", SPIFFS, "/index.html");

    server.on(
      "/update", HTTP_POST,
      []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", Update.hasError() ? "Falha!" : "Sucesso!");
        delay(1000);
        ESP.restart();
      },
      []() {
        HTTPUpload& upload = server.upload();
        if (upload.status == UPLOAD_FILE_START) {
          Serial.printf("Atualizando: %s\n", upload.filename.c_str());
          if (!Update.begin()) Update.printError(Serial);
        } else if (upload.status == UPLOAD_FILE_WRITE) {
          if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
            Update.printError(Serial);
        } else if (upload.status == UPLOAD_FILE_END) {
          if (Update.end(true))
            Serial.printf("Atualização concluída (%u bytes)\n", upload.totalSize);
        }
      });

      // --- LISTAR ARQUIVOS CSV ---
      server.on("/list", HTTP_GET, []() {
        Serial.println("Listando arquivos...");
        String json = "[";
        File root = SPIFFS.open("/");
        File file = root.openNextFile();
        bool first = true;

        while (file) {
          String name = file.name();
          if (name.endsWith(".csv")) {
            if (!first) json += ",";
            json += "{\"name\":\"" + name + "\",\"size\":" + String(file.size()) + "}";
            first = false;
          }
          file = root.openNextFile();
        }
        json += "]";
        server.send(200, "application/json", json);
      });

      // --- ROTA: Download de arquivo ---
      server.on("/download", HTTP_GET, []() {
        if (!server.hasArg("file")) {
          server.send(400, "text/plain", "Faltando parâmetro 'file'");
          return;
        }
        String path = server.arg("file");
        if (!path.startsWith("/")) path = "/" + path;

        if (!fileExists(SPIFFS, path.c_str())) {
          server.send(404, "text/plain", "Arquivo não encontrado");
          return;
        }
        File file = SPIFFS.open(path, FILE_READ);
        server.streamFile(file, "text/csv");
        file.close();
      });

      // --- ROTA: Excluir arquivo ---
      server.on("/delete", HTTP_GET, []() {
        if (!server.hasArg("file")) {
          server.send(400, "text/plain", "Faltando parâmetro 'file'");
          return;
        }
        String path = server.arg("file");
        if (!path.startsWith("/")) path = "/" + path;
        
        if (deleteFile(SPIFFS, path.c_str())) {
          server.send(200, "text/plain", "Arquivo deletado");
        } else {
          server.send(500, "text/plain", "Falha ao deletar arquivo");
        }
      });

      // --- ROTA: Teste de SPIFFS ---
      server.on("/spiffs/test", HTTP_GET, []() {
        if (testFileIO(SPIFFS, "/test.txt"))
          server.send(200, "text/plain", "Teste SPIFFS OK");
        else
          server.send(500, "text/plain", "Falha no teste SPIFFS");
      });


    server.begin();
  }

  void pararOTA_AP() {
    if (!otaAtivo) return;
    otaAtivo = false;

    server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("OTA desativado");
  }

//FIm OTA
void setup() {
  // Inicializa o pino de ligar o aparelho
  pinMode(ONOFF, OUTPUT);
  digitalWrite(ONOFF, HIGH);
  // Configuração do pino LED como saída
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;     // sem interrupção
  io_conf.mode = GPIO_MODE_OUTPUT;           // saída
  io_conf.pin_bit_mask = (1ULL << LED_PIN);  // seleciona o pino
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  // Garante que comece apagado
  GPIO.out_w1tc = (1 << LED_PIN);

  // Inicializa o pino do sensor IR
  pinMode(SENSOR_IR_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(SENSOR_IR_PIN), count, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_IR_PIN), countWhiteStripe, RISING);
  setupButtons();
  EEPROM.begin(EEPROM_SIZE);
  // Inicia a comunicação I2C com os pinos corretos
  Wire.begin(21, 22);
  // SDA = 21, SCL = 22
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Falha ao iniciar OLED"));
    while (true);
  }
  display.clearDisplay();
  // Inicializa o timer com resolução de 1us (1MHz)
  Serial.begin(115200);
  while (!Serial);

  iniciarSpiffs();
  
  #ifdef GERAHTML
    deleteFile(SPIFFS, "/index.html");
    if(!fileExists(SPIFFS, "/index.html")){
      if (writeFile(SPIFFS, "/index.html", htmlContent)) {
        Serial.println("✅ Arquivo index.html salvo com sucesso!");
      } else {
        Serial.println("❌ Erro ao salvar o arquivo!");
      }
    }
  #endif
  iniciarSPIFlash();
  identificarJEDEC();
  exibirImagemDaFlash(0x00000, 128, 64, 0, 0);
  delay(2000);
  //Inicializa acelerometro após delay para reconhecer com certeza
    adxlAvailable = accel.begin();
    if (adxlAvailable) {
      accel.setRange(ADXL345_RANGE_4_G);
      AcelAvailable = true;
    }
    lsmAvailable = lsm6ds.begin_I2C();  // inicializa via I2C
    if (lsmAvailable) {
      lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);  // igual ao ADXL345
      AcelAvailable = true;
    }
  display.clearDisplay();
  exibirImagemDaFlash(0x0041F, 128, 32, 0, 20);
  if (EEPROM.read(0) == 0xFF) {
    Serial.println("EEPROM vazia. Gravando valores padrao...");
    // Defina seus valores padrão aqui
    setValor(dadosConfig, "IDIOMA", "1");
    setValor(dadosConfig, "TIMECALIB", "10");
    setValor(dadosConfig, "FPM", "1200");
    setValor(dadosConfig, "DUTY", "4");
    setValor(dadosConfig, "CALIBFATRPM", "1.000");
    setValor(dadosConfig, "INDEXFILE", "1");
    setValor(dadosConfig, "SAMPLERATE", "200");
    setValor(dadosConfig, "FFT", "512");
    // Salva os valores padrão na EEPROM
    salvarDadosEEPROM(dadosConfig);
  } else {
    // Se a EEPROM nao estiver em branco, carrega os dados
    carregarDadosEEPROM(dadosConfig);
    lastDadosConfig = dadosConfig;
  }
  numIdioma = getValor(dadosConfig, "IDIOMA").toInt();
  idioma = getSiglaIdioma(numIdioma);
  timeCalib = getValor(dadosConfig, "TIMECALIB").toInt();
  fpm = getValor(dadosConfig, "FPM").toInt();
  dutyCycle = getValor(dadosConfig, "DUTY").toInt();
  CalibFat = getValor(dadosConfig, "CALIBFATRPM").toFloat();
  indexFile = getValor(dadosConfig, "INDEXFILE").toInt();
  SAMPLE_RATE = getValor(dadosConfig, "SAMPLERATE").toInt();
  FFT_SIZE = getValor(dadosConfig, "FFT").toInt();

  delay(1000);
  carregarArquivoParcial(displayConfig, 0x006DD, 0x001FFF, idioma);
  delay(1000);
  display.clearDisplay();

  //Stroboscópio
  // Inicializa o timer com resolução de 1us (1MHz)
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, onTimer);
  timerAlarm(timer, STB_partTime, true, 0);
}
void loop() {
  handleInput();
  //==== STROBOSCOPIO ====
  if (STB_calc) {
    updateValues();
  }
  //==== FIM STROBOSCOPIO ====

  if (AcelAvailable) {
    updateMeasurement();
    updateCalibration();
  }
  if (inMenu) {
    drawMenu();
    vibroState = VibroState::VIBRO_HOME;
    valInEncoder = 0;
    modeConfig = 0;
    modeCalibFatRpm=0;
    if (otaAtivo) pararOTA_AP();
    //==== STROBOSCOPIO ====
    if (lanterna) {
      timer = timerBegin(1000000);
      timerAttachInterrupt(timer, onTimer);
      timerAlarm(timer, STB_partTime, true, 0);
      lanterna = false;
    }
    GPIO.out_w1tc = (1 << LED_PIN);  //LOW
    STB_outputEnabled = false;
    modeFreq = 0;
    STB_phaseDegrees = 0;
    //==== FIM STROBOSCOPIO ====

    // === Lógica do Encoder para navegação do menu principal ===
    static long lastEncoderPosition = 0;
    long newPosition = encoder.read() / Sensib_Encoder;  // Ajuste o divisor para a sensibilidade que preferir
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
          //Comandos para alterar o valor usando o Encoder
          if (!STB_outputEnabled) {
            dutyCycle = getValor(dadosConfig, "DUTY").toInt();
            STB_calc = true;
          }
          STB_outputEnabled = true;
          long newPos = encoder.read() / Sensib_Encoder;
          // Dividido por 8 para reduzir sensibilidade
          if (modeFreq == 1) {
            int delta = newPos - lastEncoderPos;

            if (delta != 0) {
              // Atualiza fase em 1 grau por tick
              if (delta > 0) {
                STB_phaseDegrees = (STB_phaseDegrees + 1) % 360;
              } else {
                STB_phaseDegrees--;
                if (STB_phaseDegrees < 0) STB_phaseDegrees += 360;
              }

              lastEncoderPos = newPos;

              // Recalcula tempos do estroboscópio
              STB_calc = true;
              updateValues();  // força atualização imediata

              // Aplica fase imediatamente no próximo pulso
              portENTER_CRITICAL(&STB_timerMux);
              STB_firstPulse = true;                 // sinaliza que a fase deve ser aplicada
              timerStop(timer);                      // pausa temporariamente o timer
              timerWrite(timer, 0);                  // reseta o contador
              timerAttachInterrupt(timer, onTimer);  // reconecta a ISR (compatível 3.3.1)
              timerStart(timer);                     // reinicia o timer
              STB_outputEnabled = true;              // garante saída ativa
              portEXIT_CRITICAL(&STB_timerMux);
            }
          } else if (modeFreq == 2) {
            int delta = newPos - lastEncoderPos;
            if (delta != 0) {
              dutyCycle = constrain(dutyCycle + delta, minDuty, maxDuty);
              lastEncoderPos = newPos;
              setValor(dadosConfig, "DUTY", String(dutyCycle));
              STB_calc = true;
            }
          } else {
            int delta = newPos - lastEncoderPos;
            if (valInEncoder == 1) delta *= 10;
            else if (valInEncoder == 2) delta *= 100;
            if (delta != 0) {
              fpm = constrain(fpm + delta, minFPM, maxFPM);
              lastEncoderPos = newPos;
              setValor(dadosConfig, "FPM", String(fpm));
              STB_calc = true;
            }
          }
          break;
        }
      case Mode::RPM:
        {
          loadRPM();
          break;
        }
      case Mode::LANTERN:
        lanterna = true;
        STB_outputEnabled = false;       // flag de segurança
        timerDetachInterrupt(timer);     // desliga o timer (não chama mais a ISR)
        dutyCycle = 100;                 // força duty a 100%
        GPIO.out_w1ts = (1 << LED_PIN);  // mantém LED aceso fixo
        break;
      case Mode::TEST:
        {
          static unsigned long ultimoTempo = 0;
          static float fpmAtual = 30.0;
          static const float passo = 30.0;
          static const unsigned long intervaloTeste = 500;  // 2 segundos entre passos

          unsigned long agora = millis();

          if (fpmTest.isRunning()) {
            if (agora - ultimoTempo >= intervaloTeste) {
              TESTE_fpm = fpmAtual;
              TESTE_calc = true;  // <<< Ativa cálculo com TESTE_fpm
              updateValues();

              fpmAtual += passo;
              FreqDeTest = fpmAtual / 60.0;

              if (fpmAtual >= maxFPM) {
                fpmAtual = 30.0;  // reinicia ciclo
              }
              ultimoTempo = agora;
            }
            STB_outputEnabled = true;  // <<< Mantém LED rodando
          } else if (fpmTest.isExpired()) {
            fpmAtual = 30.0;
            FreqDeTest = fpmAtual / 60.0;
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
          long newPos = encoder.read() / Sensib_Encoder;
          int delta = newPos - lastEncoderPos;
          if (delta != 0) {
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
      case Mode::VIBROMETER:
        {
          long newPos = encoder.read() / Sensib_Encoder;
          int delta = newPos - lastEncoderPos;
          if (delta != 0) {
            if (vibroState == VibroState::VIBRO_SELECTCALIB) {
              timeCalib = constrain(timeCalib + (delta * 5), 5, 30);
              saveData = setValor(dadosConfig, "TIMECALIB", String(timeCalib));
            } else if (vibroState == VibroState::VIBRO_SELECTMEASURE) {
              if (modeMeasure == 1){
                if(SAMPLE_RATE >= 200 && SAMPLE_RATE <= 500)
                  FFT_SIZE = 512;
                else if(SAMPLE_RATE > 500 && SAMPLE_RATE < 2000)
                  FFT_SIZE = 1024;
                else if(SAMPLE_RATE >= 2000 && SAMPLE_RATE < 4000)
                  FFT_SIZE = constrain(FFT_SIZE + (delta * 512), 512, 1024);
                else if(SAMPLE_RATE >= 4000 && SAMPLE_RATE < 5000)
                  FFT_SIZE = 1024;
                else if(SAMPLE_RATE >= 5000 && SAMPLE_RATE <= 10000)
                  FFT_SIZE = constrain(FFT_SIZE + (delta * 512), 1024, 2048);
                saveData = setValor(dadosConfig, "FFT", String(FFT_SIZE));
              }else if (modeMeasure == 0){
                SAMPLE_RATE = constrain(SAMPLE_RATE + (delta * 100), 100, 10000);
                saveData = setValor(dadosConfig, "SAMPLERATE", String(SAMPLE_RATE));
              }
            }
            lastEncoderPos = newPos;
          }
          break;
        }
      case Mode::CONFIG:
        {
          if (!otaAtivo) iniciarOTA_AP();
          server.handleClient();
          
          long newPos = encoder.read() / Sensib_Encoder;
          int delta = newPos - lastEncoderPos;
          if (delta != 0) {
            if (modeConfig == 1) {
              if(modeCalibFatRpm == 1) step = 0.010;
              else if (modeCalibFatRpm == 2) step = 0.100;
              else step = 0.001;
              step = delta*step;
              CalibFat = constrain(CalibFat + step, minVal, maxVal);
              setValor(dadosConfig, "CALIBFATRPM", String(CalibFat));
            }
            lastEncoderPos = newPos;
          }
          break;
        }
    }
  }
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
      currentMode = selectedMode;
      updateValuesRec();
    }
  }
  if (checkButtonDebounce(BUTTON_SET, lastSetState, lastDebounceTimeSet, 50000)) {
    if (inMenu) {
      selectedMode = currentMode;
      inMenu = false;
    } else {
      if (selectedMode == Mode::VIBROMETER) {
        switch (vibroState) {
          //Clique SET para iniciar
          case VibroState::VIBRO_HOME:
            vibroState = VibroState::VIBRO_SELECTCALIB;
            break;
          case VibroState::VIBRO_SELECTCALIB:
            vibroState = VibroState::VIBRO_CALIBRATING;
            startCalibration(getValor(dadosConfig, "TIMECALIB").toInt());
            break;
          case VibroState::VIBRO_CALIBRATING:
            //a função startCalibration que muda para próxima tela vibroState = VibroState::VIBRO_SELECTMEASURE;
            break;
          case VibroState::VIBRO_SELECTMEASURE:
            vibroState = VibroState::VIBRO_MEASURING;
            startMeasurement();
            break;
          case VibroState::VIBRO_MEASURING:
            //a função startMeasurement que muda para próxima tela vibroState = VibroState::VIBRO_MEASURERESULT;
            break;
          case VibroState::VIBRO_MEASURERESULT:
            vibroState = VibroState::VIBRO_ABOUT;
            break;
          case VibroState::VIBRO_ABOUT:
            vibroState = VibroState::VIBRO_HOME;
            break;
          default:
            // Não faz nada em outros estados do vibrometro
            break;
        }
      } else if (selectedMode == Mode::FREQUENCY) {
        if(modeFreq == 0){
          valInEncoder++;
          if (valInEncoder >= 3) {
            valInEncoder = 0;
          }
        }
      } else if (selectedMode == Mode::RPM) {
        if (rpmValue >= 30) {
          fpm = rpmValue;
          //FPM recebe o valor de RPM
          setValor(dadosConfig, "FPM", String(fpm));
          STB_calc = true;
          msgTimer.startTimer(2);  //Inicia a contagem para exibir a mensagem de gravando por 2 segundos
        }
      } else if (selectedMode == Mode::TEST) {
        fpmTest.startTimer(25);
      } else if (selectedMode == Mode::LANTERN || currentMode == Mode::ABOUT) {
        inSubmenu = false;
      } else if (selectedMode == Mode::CONFIG) {
        if (modeConfig == 1){
            modeCalibFatRpm++;
            if (modeCalibFatRpm >= 3) {
              modeCalibFatRpm = 0;
            }
        } else if (modeConfig == 2) { //reset de fabrica
            Serial.println("EEPROM vazia. Gravando valores padrao...");
            // Defina seus valores padrão aqui
            setValor(dadosConfig, "IDIOMA", "1");
            setValor(dadosConfig, "TIMECALIB", "10");
            setValor(dadosConfig, "FPM", "1200");
            setValor(dadosConfig, "DUTY", "4");
            setValor(dadosConfig, "CALIBFATRPM", "1.000");
            setValor(dadosConfig, "INDEXFILE", "1");
            setValor(dadosConfig, "SAMPLERATE", "200");
            setValor(dadosConfig, "FFT", "512");
            // Salva os valores padrão na EEPROM
            salvarDadosEEPROM(dadosConfig);
        }
      } else {
        inSubmenu = !inSubmenu;
      }
    }
  }
  if (checkButtonDebounce(BUTTON_HALF, lastDoubleState, lastDebounceTimeDouble, 50000)) {
    if (inMenu) {
      currentMode = static_cast<Mode>((static_cast<int>(currentMode) + 1) % static_cast<int>(Mode::NUM_MODES));
    } else if (selectedMode == Mode::HOME) {
      if (topLineIndex < totalLines - lineShowMsg) {  // lineShowMsg = linhas cabem na tela
        topLineIndex++;
      }
    } else if (currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_SELECTCALIB) {
      timeCalib = constrain(timeCalib - 5, 5, 30);
    } else if (currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_SELECTMEASURE) {
      if (modeMeasure == 1){
        FFT_SIZE = constrain(FFT_SIZE + 512, 512, 2048);
        saveData = setValor(dadosConfig, "FFT", String(FFT_SIZE));
      } else if (modeMeasure == 0){
        SAMPLE_RATE-= 100;
        if (SAMPLE_RATE < 100) {
          SAMPLE_RATE = 10000;
        }
        saveData = setValor(dadosConfig, "SAMPLERATE", String(SAMPLE_RATE));
      }
    } else if (currentMode == Mode::CONFIG) {
      if (modeConfig == 0) {
        numIdioma = (numIdioma - 1 + 1) % 4 + 1;
        setValor(dadosConfig, "IDIOMA", String(numIdioma));
      } else if (modeConfig == 1) {
        if(modeCalibFatRpm == 1) step = 0.010;
        else if (modeCalibFatRpm == 2) step = 0.100;
        else step = 0.001;
        CalibFat -= step;
        if (CalibFat < minVal) CalibFat = maxVal;
        setValor(dadosConfig, "CALIBFATRPM", String(CalibFat));
      }
    } else {
      if (selectedMode == Mode::FREQUENCY && !inSubmenu) {
        adjustFPM(0.5);
        setValor(dadosConfig, "FPM", String(fpm));
        STB_calc = true;
      }
    }
  }
  if (checkButtonDebounce(BUTTON_DOUBLE, lastHalfState, lastDebounceTimeHalf, 50000)) {
    if (inMenu) {
      currentMode = static_cast<Mode>((static_cast<int>(currentMode) - 1 + static_cast<int>(Mode::NUM_MODES)) % static_cast<int>(Mode::NUM_MODES));
    } else if (selectedMode == Mode::HOME) {
      if (topLineIndex > 0) {
        topLineIndex--;
      }
    } else if (currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_SELECTCALIB) {
      timeCalib = constrain(timeCalib + 5, 5, 30);
    } else if (currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_SELECTMEASURE) {
      if (modeMeasure == 1){
        FFT_SIZE = constrain(FFT_SIZE + 512, 512, 2048);
        saveData = setValor(dadosConfig, "FFT", String(FFT_SIZE));
      } else if (modeMeasure == 0){
        SAMPLE_RATE+= 100;
        if (SAMPLE_RATE > 10000) {
          SAMPLE_RATE = 100;
        }
        saveData = setValor(dadosConfig, "SAMPLERATE", String(SAMPLE_RATE));
      }
    } else if (currentMode == Mode::CONFIG) {
      if (modeConfig == 0) {
        numIdioma = (numIdioma - 1 - 1 + 4) % 4 + 1;
        setValor(dadosConfig, "IDIOMA", String(numIdioma));
      } else if (modeConfig == 1) {
        if(modeCalibFatRpm == 1) step = 0.010;
        else if (modeCalibFatRpm == 2) step = 0.100;
        else step = 0.001;
        CalibFat += step;
        if (CalibFat > maxVal) CalibFat = minVal;
        setValor(dadosConfig, "CALIBFATRPM", String(CalibFat));
      }
    } else {
      if (selectedMode == Mode::FREQUENCY && !inSubmenu) {
        adjustFPM(2.0);
        setValor(dadosConfig, "FPM", String(fpm));
        STB_calc = true;
      }
    }
  }
  if (checkButtonDebounce(BUTTON_ENC, lastEncState, lastDebounceTimeEnc, 50000)) {
    if (selectedMode == Mode::FREQUENCY) {
      modeFreq++;
      if (modeFreq >= 3) {
        modeFreq = 0;
      }
    } else if (selectedMode == Mode::CONFIG){
      modeConfig++;
      if (modeConfig >= 3) {
        modeConfig = 0;
      }
    } else if(vibroState == VibroState::VIBRO_SELECTMEASURE){
      modeMeasure++;
      if (modeMeasure >= 2) {
        modeMeasure = 0;
      }
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
    //display.println("Agradecimentos: ");
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
    exibirImagemDaFlash(0x0063E, 32, 32, 90, 15);
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
    display.println("Fator calib RPM:");
    display.println(CalibFat,3);
    if (modeConfig == 0) display.setCursor(95, 22);
    if (modeConfig == 1) display.setCursor(95, 37);
    if (modeConfig == 2) display.setCursor(95, 47);
    display.print("<");
    if(modeCalibFatRpm == 0) display.setCursor(24, 40);
    if(modeCalibFatRpm == 1) display.setCursor(18, 40);
    if (modeCalibFatRpm == 2) display.setCursor(12, 40);
    display.println("_");
    display.println("Factory Reset");
  } else if (currentMode == Mode::FREQUENCY) {
    display.setTextSize(2);
    display.setCursor(0, 14);
    String fpmString = String(fpm);
    if(fpm < 1000 && fpm >= 100)
      fpmString = "0"+String(fpm);
    else if (fpm < 100)
      fpmString = "00"+String(fpm);
    display.print(fpmString);
    //display.print((int)fpm);
    display.setTextSize(1);
    display.setCursor(70, 22);
    display.print(getValor(displayConfig, "FPM", idioma));
    display.setCursor(0, 34);
    display.print(getValor(displayConfig, "HZ", idioma) + ": ");
    display.print(fpm / 60.0, 2);
    display.setCursor(0, 46);
    display.print(getValor(displayConfig, "PHASE", idioma) + " ");
    display.print(STB_phaseDegrees);
    display.println((char)247);
    
    display.setCursor(0, 56);
    display.print("Duty: ");
    display.print(dutyCycle);
    if (modeFreq == 0) {
      display.setCursor(95, 22);
    }
    if (modeFreq == 1) {
      display.setCursor(95, 46);
    }
    if (modeFreq == 2) {
      display.setCursor(95, 56);
    }
    display.print("<");
    if(valInEncoder == 0){
      display.setCursor(40, 25);
    }
    if(valInEncoder == 1){
      display.setCursor(27, 25);
    }
    if(valInEncoder == 2){
      display.setCursor(14, 25);
    }
    display.print("_");
  } else if (currentMode == Mode::VIBROMETER) {
    if (AcelAvailable) {
      switch (vibroState) {
        case VibroState::VIBRO_HOME:
          display.println(getValor(displayConfig, "CALIBRATE", idioma));
          break;
        case VibroState::VIBRO_SELECTCALIB:
          display.print(getValor(displayConfig, "SELECTTIME", idioma));
          display.print(timeCalib);
          display.println("s");
          display.println("");
          display.println(getValor(displayConfig, "SETCALIB", idioma));
          break;
        case VibroState::VIBRO_CALIBRATING:
          display.println(getValor(displayConfig, "CALIBRATING", idioma));
          display.println("");
          display.print(getValor(displayConfig, "RESTTIME", idioma));
          display.print(secondsLeftCalib);
          display.println("s");
          break;
        case VibroState::VIBRO_SELECTMEASURE:
          //display.print(getValor(displayConfig, "SELECTTIME", idioma));
          display.print("Sample Rate: ");
          display.println(SAMPLE_RATE);
          display.print("FFT Size: ");
          display.println(FFT_SIZE);
          display.println(getValor(displayConfig, "MSGIDLE1", idioma));
          display.println(getValor(displayConfig, "MSGIDLE2", idioma));
          if (modeMeasure == 1) display.setCursor(120, 13);
          if (modeMeasure == 0) display.setCursor(120, 21);
          display.print("<");
          break;
        case VibroState::VIBRO_MEASURING:
          display.println(getValor(displayConfig, "MEASURE", idioma));
          display.println("");
          display.println(getValor(displayConfig, "RESTTIME", idioma));
          display.println(samplesRemaining);
          break;
        case VibroState::VIBRO_MEASURERESULT:
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
        case VibroState::VIBRO_ABOUT:
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