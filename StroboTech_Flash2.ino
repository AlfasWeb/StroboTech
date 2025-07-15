#include "flash.h" // Inclui o arquivo de cabeçalho "flash.h", que provavelmente contém funções para interação com a memória flash (leitura/escrita de dados e imagens).
#include "config.h" // Inclui o arquivo de cabeçalho "config.h", que provavelmente define estruturas ou funções relacionadas à configuração do dispositivo.

#include <Wire.h> // Inclui a biblioteca Wire para comunicação I2C, usada para o display OLED e o acelerômetro ADXL345.
#include <Adafruit_GFX.h> // Inclui a biblioteca Adafruit GFX, que fornece primitivas gráficas comuns (texto, linhas, formas) para displays.
#include <Adafruit_SSD1306.h> // Inclui a biblioteca específica para o driver do display OLED SSD1306.

#include <Adafruit_Sensor.h> // Inclui a biblioteca genérica de sensores da Adafruit, base para outros drivers de sensores.
#include <Adafruit_ADXL345_U.h> // Inclui a biblioteca específica para o acelerômetro ADXL345 da Adafruit.
#define ENCODER_DO_NOT_USE_INTERRUPTS // Define uma macro para instruir a biblioteca Encoder a não usar interrupções (pode ser útil para evitar conflitos ou simplificar o código em alguns microcontroladores).
#include <Encoder.h> // Inclui a biblioteca Encoder, usada para ler o encoder rotativo conectado aos pinos digitais.
#include <arduinoFFT.h> // Inclui a biblioteca arduinoFFT para realizar a Transformada Rápida de Fourier (FFT) em dados de aceleração.

// ==== Pinos ====
#define ENCODER_PIN_A 32 // Define o pino GPIO 32 para a fase A do encoder rotativo.
#define ENCODER_PIN_B 33 // Define o pino GPIO 33 para a fase B do encoder rotativo.
#define BUTTON_MENU 25 // Define o pino GPIO 25 para o botão de navegação do menu (próximo modo).
#define BUTTON_SET 14 // Define o pino GPIO 14 para o botão de seleção ou confirmação.
#define BUTTON_DOUBLE 26 // Define o pino GPIO 26 para um botão de função "dobrar" (ex: FPM x2, rolar para baixo no menu).
#define BUTTON_HALF 27 // Define o pino GPIO 27 para um botão de função "metade" (ex: FPM /2, rolar para cima no menu).
#define BUTTON_ENC 17 // Define o pino GPIO 17 para o botão de clique do encoder.
#define LED_PIN 2 // Define o pino GPIO 2 para o LED de alta potência (usado para o estroboscópio e lanterna).
#define SENSOR_IR_PIN 4 // Define o pino GPIO 4 para o sensor infravermelho, usado para medição de RPM.
/*
GPIO Observações
21-22 -SDA, SCL // Pinos GPIO 21 e 22 são comumente usados para comunicação I2C (SDA e SCL) em placas ESP32.
5,23,18,19 - SPI // Pinos GPIO 5, 23, 18, 19 são comumente usados para comunicação SPI (Serial Peripheral Interface).
*/

// ==== Display ====
#define SCREEN_WIDTH 128 // Define a largura do display OLED em pixels.
#define SCREEN_HEIGHT 64 // Define a altura do display OLED em pixels.
#define OLED_RESET -1 // Define o pino de reset do OLED. -1 indica que não há um pino de reset dedicado, ou ele é gerenciado por software/hardware.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Cria um objeto 'display' da classe Adafruit_SSD1306, configurando suas dimensões, a interface I2C (&Wire) e o pino de reset.
// =================
// ==== Encoder e ADXL ====
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B); // Cria um objeto 'encoder' da classe Encoder, associando-o aos pinos definidos para as fases A e B.
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(123); // Cria um objeto 'accel' da classe Adafruit_ADXL345_Unified com um ID de sensor arbitrário (123).
bool adxlAvailable = false; // Variável booleana para indicar se o acelerômetro ADXL345 foi detectado e inicializado com sucesso.

//==== Inicialização do medidor de FFT ====
#define SAMPLE_RATE 500 // Define a taxa de amostragem em Hertz (Hz) para a coleta de dados de aceleração para a FFT.
#define FFT_SIZE 512 // Define o tamanho da FFT (número de amostras). Deve ser uma potência de 2 (ex: 256, 512, 1024) para o algoritmo da FFT.
double accBuffer[FFT_SIZE]; // Array para armazenar os valores de aceleração (componente real para a FFT).
double timeBuffer[FFT_SIZE]; // Array para a componente imaginária da FFT (preenchido com zeros, pois os dados de aceleração são reais).
ArduinoFFT<double> FFT = ArduinoFFT<double>(accBuffer, timeBuffer, FFT_SIZE, SAMPLE_RATE); // Cria um objeto 'FFT' da classe ArduinoFFT, inicializando-o com os buffers, tamanho e taxa de amostragem.

// ==== Menu ====
enum class Mode { HOME, FREQUENCY, RPM, LANTERN, VIBROMETER, TEST, ABOUT, CONFIG, NUM_MODES}; // Enumeração que define os diferentes modos de operação do dispositivo.
Mode currentMode = Mode::HOME; // Variável que armazena o modo atualmente selecionado no menu (o que o usuário está vendo e navegando).
Mode selectedMode = Mode::HOME; // Variável que armazena o modo que foi *selecionado* para execução e está ativo no momento.
bool inMenu = true; // Flag booleana que indica se o sistema está na interface de seleção de modos (true) ou em um modo específico (false).
bool inSubmenu = false; // Flag booleana que indica se o sistema está em um submenu de um modo (ex: ajuste de fase no estroboscópio).
bool inEncoder = false; // Flag booleana que indica se o encoder está operando em um modo de sensibilidade aumentada (ex: x10).

// ==== Variáveis do Vibrometro ====
enum class VibroState {VIBRO_HOME, VIBRO_IDLE, VIBRO_CALIB, VIBRO_CONFIG, VIBRO_MEASURE, VIBRO_RESULT}; // Enumeração que define os diferentes estados dentro do modo Vibrometro.
VibroState vibroState = VibroState::VIBRO_HOME; // Variável que armazena o estado atual da máquina de estados do Vibrometro.
// ==== Medição ====
float ax, ay, az; // Variáveis para armazenar as componentes de aceleração nos eixos X, Y e Z (medidas pelo ADXL345).
float aRMS, aPeak, stdDev, vRMS, freqDominant; // Variáveis para armazenar os resultados calculados da análise de vibração: RMS de aceleração, Pico de aceleração, Desvio padrão, RMS de velocidade e Frequência Dominante.
float velocitySum = 0; // Variável para acumular valores de aceleração para o cálculo da velocidade RMS.

unsigned long lastSampleTime = 0; // Variável para registrar o tempo (em microssegundos) da última amostra de aceleração coletada.
int sampleCount = 0; // Contador de amostras coletadas para preencher o buffer da FFT.

bool isMeasuring = false; // Flag booleana que indica se uma medição de vibração está em andamento.
bool isCalibrating = false; // Flag booleana que indica se o processo de calibração do acelerômetro está em andamento.

unsigned long measureStartTime = 0; // Tempo (em milissegundos) em que a medição de vibração foi iniciada.
unsigned long measureDuration = 0; // Duração total (em milissegundos) planejada para a medição de vibração.
unsigned long calibrationStartTime = 0; // Tempo (em milissegundos) em que a calibração foi iniciada.
unsigned long calibrationDuration = 0; // Duração total (em milissegundos) planejada para a calibração.

int secondsLeft = 0; // Segundos restantes para a medição de vibração atual (para exibição no display).
int secondsLeftCalib = 0; // Segundos restantes para a calibração atual (para exibição no display).

float offsetX = 0, offsetY = 0, offsetZ = 0; // Valores de offset (viés) para cada eixo do acelerômetro, calculados durante a calibração para remover o efeito da gravidade e outros desvios.
//-------------------------------------
// ==== Variáveis de Gravação do tempo de vibração ====
int timeMeasure = 30; // Duração padrão da medição de vibração em segundos.
int timeCalib = 15; // Duração padrão da calibração em segundos.

// ==== Variáveis do RPM ====
#define TEMPO_LEITURA_RPM 1000 // Define o intervalo em milissegundos (1 segundo) para calcular e atualizar o valor de RPM.
unsigned long ultimaLeitura = 0; // Variável para registrar o tempo (em milissegundos) da última vez que o RPM foi calculado.
int contagemPulsos = 0; // Contador de pulsos detectados pelo sensor IR em um determinado intervalo de tempo.
bool estadoAnterior = LOW; // Variável booleana para armazenar o estado anterior do pino do sensor IR (usado para detecção de borda de subida/descida).
float rpmValue = 0; // Variável para armazenar o valor de RPM calculado e exibido.

bool Lant_calc = false; // Flag booleana que indica se o modo Lanterna está ativo e se seus parâmetros precisam ser recalculados.
bool TESTE_calc = false; // Flag booleana que indica se o modo de Teste (varredura de FPM) está ativo e se seus parâmetros precisam ser recalculados.
float TESTE_fpm = 0; // Variável que armazena o valor de FPM atual durante o modo de Teste.
//-------------------------------------
// Lista de mensagens
String lines[] = { // Array de strings que contém mensagens de agradecimento ou informações para o modo "Sobre".
  "Agradecimento:",
  "Evandro Padilha",
  "Renato",
  "Vitor Santarosa",
  "Alex Penteado",
  "Bruno",
  "Epaminondas",
  "Fernando",
  "Fabio Camarinha",
  "Gabriela"
};
const int totalLines = sizeof(lines) / sizeof(lines[0]); // Calcula o número total de linhas na lista de mensagens.
int topLineIndex = 0; // Índice da linha superior visível atualmente no display (usado para rolagem de texto).
int lineShowMsg = 5; // Número de linhas de mensagem que podem ser exibidas simultaneamente na tela.
// ==============

// ==== Temporizador ====
struct TimerMicros { // Definição de uma estrutura para criar temporizadores não bloqueantes com precisão de microssegundos.
  unsigned long start; // O tempo (em microssegundos) em que o timer foi iniciado.
  unsigned long duration; // A duração (em microssegundos) para a qual o timer foi configurado.
  // Inicia o timer com uma duração específica.
  void startTimer(unsigned long d) {
    duration = d; // Define a duração do timer.
    start = micros(); // Registra o tempo atual como o ponto de partida do timer.
  }
  // Verifica se o tempo decorrido desde o início do timer atingiu ou excedeu sua duração.
  bool isExpired() {
    return (micros() - start) >= duration; // Retorna true se o timer expirou.
  }
  // Verifica se o timer ainda está ativo (não expirou).
  bool isRunning() {
    return (micros() - start) < duration; // Retorna true se o timer ainda está contando.
  }
};
TimerMicros msgTimer; // Cria uma instância de TimerMicros para gerenciar o tempo de exibição de mensagens temporárias.
TimerMicros fpmTest; // Cria uma instância de TimerMicros para gerenciar a duração do ciclo no modo de teste de FPM.

// ==== Vetores da Flash ====
std::vector<Config> dadosConfig; // Vetor para armazenar objetos 'Config' que representam dados de configuração geral do dispositivo, carregados ou salvos na flash.
std::vector<Config> displayConfig; // Vetor para armazenar objetos 'Config' que representam strings de texto para o display (nomes de modos, mensagens, etc.), geralmente dependentes do idioma.
int numIdioma = 1; // Variável que armazena o número inteiro correspondente ao idioma selecionado (ex: 1 para Português, 2 para Inglês).
String idioma = ""; // Variável que armazena a sigla do idioma atual (ex: "PT", "EN", "ES", "FR"), usada para buscar textos localizados.
bool saveData = false; // Flag booleana que indica se há dados de configuração pendentes para serem salvos na memória flash.
// ==============

// ==== Variáveis do Modo Estroboscópio ====
float fpm = 300; // Valor inicial de FPM (Flashes Por Minuto) para o modo estroboscópio.
long STB_lastEncoderPos = 0; // Armazena a última posição lida do encoder para controlar o ajuste de FPM ou fase no modo estroboscópio.

float STB_phaseDegrees = 0.0; // Ajuste de fase em graus para o estroboscópio, permitindo sincronizar o flash com um evento específico.
unsigned long STB_partTime = 1000000; // Duração de meio ciclo do estroboscópio em microssegundos (o tempo que o LED fica ON ou OFF).
unsigned long STB_phaseDelayMicros = 0; // Atraso de fase calculado em microssegundos, aplicado no início de cada ciclo para a fase.
bool STB_calc = true; // Flag booleana que indica se os parâmetros de tempo do estroboscópio precisam ser recalculados (ex: após mudança de FPM ou fase).
bool STB_firstPulse = true; // Flag booleana para indicar o primeiro pulso de um ciclo, onde o atraso de fase é aplicado.
bool STB_outputEnabled = false; // Flag booleana que controla se a saída do LED do estroboscópio está fisicamente ativa.

hw_timer_t *timer = NULL; // Ponteiro para um objeto de timer de hardware do ESP32, usado para gerar pulsos precisos para o estroboscópio.
portMUX_TYPE STB_timerMux = portMUX_INITIALIZER_UNLOCKED; // Mutex (mecanismo de bloqueio) para proteger variáveis compartilhadas entre a rotina de interrupção (ISR) do timer e o código principal.
volatile bool STB_pulseState = false; // Variável volátil booleana que armazena o estado atual do pulso do LED (HIGH ou LOW), acessada e modificada pela ISR.

const int minFPM = 30; // Limite mínimo de FPM que pode ser configurado.
const int maxFPM = 40000; // Limite máximo de FPM que pode ser configurado.
// ==== Função de interrupção do timer. Alterna o estado do LED e aplica o atraso de fase na primeira chamada. ====
void IRAM_ATTR onTimer() { // Define a função 'onTimer' como uma Rotina de Serviço de Interrupção (ISR). IRAM_ATTR coloca a função na RAM de alta velocidade para garantir execução rápida e consistente.
  static bool phaseApplied = false; // Variável estática interna à ISR para controlar se o atraso de fase já foi aplicado neste pulso específico.
  portENTER_CRITICAL_ISR(&STB_timerMux); // Entra em uma seção crítica para proteger variáveis globais ou voláteis de acesso concorrente pela ISR e pelo loop principal.

  if (!STB_outputEnabled) { // Verifica se a saída do estroboscópio está desabilitada.
    digitalWrite(LED_PIN, LOW); // Se desabilitada, garante que o LED esteja desligado.
    portEXIT_CRITICAL_ISR(&STB_timerMux); // Sai da seção crítica.
    return; // Sai da ISR.
  }

  // Lógica para aplicar o atraso de fase na primeira vez que o timer dispara para um novo ciclo de estroboscópio.
  if (STB_firstPulse && !phaseApplied && STB_phaseDelayMicros > 0) { // Se for o primeiro pulso, a fase não foi aplicada e há um atraso de fase configurado.
    timerAlarm(timer, STB_phaseDelayMicros, true, 0); // Reconfigura o timer para disparar após o 'STB_phaseDelayMicros' (atraso de fase), repetindo a interrupção.
    phaseApplied = true; // Marca que o atraso de fase foi aplicado.
  } else { // Se não for o primeiro pulso ou o atraso de fase já foi aplicado.
    STB_pulseState = !STB_pulseState; // Inverte o estado do pulso (de HIGH para LOW ou de LOW para HIGH).
    digitalWrite(LED_PIN, STB_pulseState); // Define o estado do LED de acordo com 'STB_pulseState'.
    timerAlarm(timer, STB_partTime, true, 0); // Reconfigura o timer para o próximo intervalo de 'STB_partTime' (metade do ciclo FPM).
    STB_firstPulse = false; // Reseta a flag 'STB_firstPulse' para que o atraso de fase não seja aplicado novamente até um novo ciclo.
    phaseApplied = false; // Reseta a flag 'phaseApplied'.
  }

  portEXIT_CRITICAL_ISR(&STB_timerMux); // Sai da seção crítica.
}

// Atualiza os valores com base na entrada de FPM. Recalcula os tempos de ciclo e atraso de fase.
void updateValues() {
  if (STB_calc) { // Se a flag 'STB_calc' for verdadeira (indicando que os parâmetros do estroboscópio normal precisam ser recalculados).
    unsigned long cycleTimeMicros = 60000000UL / fpm; // Calcula o tempo total de um ciclo completo em microssegundos (60 segundos * 1.000.000 micros/segundo dividido por FPM).
    STB_partTime = cycleTimeMicros / 2; // Define 'STB_partTime' como metade do tempo total do ciclo (duração de ON ou OFF do LED).
    STB_phaseDelayMicros = (STB_phaseDegrees / 360.0) * cycleTimeMicros; // Calcula o atraso de fase em microssegundos, baseado nos graus de fase e no tempo total do ciclo.
    STB_firstPulse = true; // Define 'STB_firstPulse' como true para garantir que o atraso de fase seja aplicado no próximo ciclo.
    STB_calc = false; // Reseta a flag 'STB_calc'.
  } else if (Lant_calc) { // Se a flag 'Lant_calc' for verdadeira (indicando que o modo Lanterna está ativo e precisa de recálculo).
    unsigned long cycleTimeMicros = 60000000UL / 7200; // Define um FPM fixo de 7200 para a lanterna (um valor alto para que o LED pareça estar constantemente ligado).
    STB_partTime = cycleTimeMicros / 2; // Define 'STB_partTime' com base nesse FPM.
    STB_phaseDelayMicros = (STB_phaseDegrees / 360.0) * cycleTimeMicros; // Recalcula o atraso de fase (embora a fase seja menos relevante para uma lanterna constante).
    STB_firstPulse = true; // Define 'STB_firstPulse' como true.
  } else if (TESTE_calc) { // Se a flag 'TESTE_calc' for verdadeira (indicando que o modo de Teste está ativo e precisa de recálculo).
    unsigned long cycleTimeMicros = 60000000UL / TESTE_fpm; // Calcula o tempo total do ciclo usando o FPM do modo de Teste.
    STB_partTime = cycleTimeMicros / 2; // Define 'STB_partTime' com base no FPM de Teste.
    STB_phaseDelayMicros = (STB_phaseDegrees / 360.0) * cycleTimeMicros; // Recalcula o atraso de fase para o modo de Teste.
    STB_firstPulse = true; // Define 'STB_firstPulse' como true.
  }
}

// ==== Ajusta o FPM multiplicando-o por um fator ====
void adjustFPM(float factor) {
  fpm = constrain(fpm * factor, minFPM, maxFPM); // Multiplica o valor atual de 'fpm' pelo 'factor' e garante que o resultado esteja dentro dos limites 'minFPM' e 'maxFPM'.
}

// Função para verificar o pressionamento de um botão com debounce (eliminação de ruído)
bool checkButtonDebounce(uint8_t pin, bool &lastState, unsigned long &lastDebounceTime, unsigned long debounceDelayMicros) {
  bool currentState = digitalRead(pin); // Lê o estado atual do pino do botão.
  unsigned long now = micros(); // Obtém o tempo atual em microssegundos.
  // Verifica se houve uma transição de estado de HIGH para LOW (botão pressionado, assumindo pull-up)
  // E se o tempo decorrido desde o último evento de debounce válido é maior que o 'debounceDelayMicros'.
  if (currentState == LOW && lastState == HIGH && (now - lastDebounceTime > debounceDelayMicros)) {
    lastDebounceTime = now; // Atualiza o tempo do último evento de debounce válido.
    lastState = currentState; // Atualiza o estado anterior do botão para o estado atual.
    return true; // Retorna true para indicar que um clique válido foi detectado.
  }
  lastState = currentState; // Atualiza o estado anterior do botão para o estado atual (mesmo que não tenha sido um clique válido).
  return false; // Retorna false se nenhum clique válido foi detectado.
}
//-------------------------------------
// ==== Funçoes do Vibrometro calibrar e medir ====
// Inicia calibração com tempo em segundos
void startCalibration(int durationSeconds) {
  isCalibrating = true; // Define a flag 'isCalibrating' como verdadeira para iniciar o processo de calibração.
  calibrationDuration = durationSeconds * 1000UL; // Converte a duração em segundos para milissegundos e armazena em 'calibrationDuration'.
  calibrationStartTime = millis(); // Registra o tempo atual (em milissegundos) como o início da calibração.
  offsetX = 0; // Zera a soma do offset X.
  offsetY = 0; // Zera a soma do offset Y.
  offsetZ = 0; // Zera a soma do offset Z.
  sampleCount = 0; // Zera o contador de amostras coletadas para a calibração.
  secondsLeftCalib = durationSeconds; // Define os segundos restantes para a calibração (para exibição).
  vibroState = VibroState::VIBRO_CALIB; // Muda o estado do vibrometro para 'VIBRO_CALIB', indicando que a calibração está em andamento.
  Serial.println("Iniciando calibração..."); // Imprime uma mensagem no Serial Monitor indicando o início da calibração.
}

// Atualiza o processo de calibração do acelerômetro, coletando amostras e calculando os offsets médios.
void updateCalibration() {
  if (!isCalibrating) return; // Se 'isCalibrating' for falsa, sai da função imediatamente.

  unsigned long now = millis(); // Obtém o tempo atual em milissegundos.
  unsigned long elapsed = now - calibrationStartTime; // Calcula o tempo decorrido desde o início da calibração.
  unsigned long remaining = (calibrationDuration > elapsed) ? (calibrationDuration - elapsed) : 0; // Calcula o tempo restante da calibração.
  secondsLeftCalib = remaining / 1000; // Converte o tempo restante para segundos.

  sensors_event_t event; // Cria um objeto 'event' para armazenar os dados de aceleração.
  accel.getEvent(&event); // Obtém os dados de aceleração do ADXL345.
  offsetX += event.acceleration.x; // Acumula a leitura X para o cálculo do offset.
  offsetY += event.acceleration.y; // Acumula a leitura Y para o cálculo do offset.
  offsetZ += event.acceleration.z; // Acumula a leitura Z para o cálculo do offset.
  sampleCount++; // Incrementa o contador de amostras.

  if (elapsed >= calibrationDuration) { // Se o tempo de calibração expirou.
    offsetX /= sampleCount; // Calcula o offset médio para o eixo X.
    offsetY /= sampleCount; // Calcula o offset médio para o eixo Y.
    offsetZ /= sampleCount; // Calcula o offset médio para o eixo Z.
    isCalibrating = false; // Define 'isCalibrating' como falsa, indicando que a calibração terminou.
    vibroState = VibroState::VIBRO_IDLE; // Muda o estado do vibrometro para 'VIBRO_IDLE' (ocioso após calibração).
    Serial.println("Calibração concluída!"); // Imprime mensagem de conclusão.
    Serial.print("Offset X: "); Serial.println(offsetX); // Imprime o offset X calculado.
    Serial.print("Offset Y: "); Serial.println(offsetY); // Imprime o offset Y calculado.
    Serial.print("Offset Z: "); Serial.println(offsetZ); // Imprime o offset Z calculado.
  }
}
//vibroState = VibroState::VIBRO_CONFIG; // Comentário indicando um possível estado futuro ou transição.
// Inicia uma medição de vibração com tempo em segundos
void startMeasurement(int durationSeconds) {
  isMeasuring = true; // Define a flag 'isMeasuring' como verdadeira para iniciar a medição.
  measureStartTime = millis(); // Registra o tempo atual como o início da medição.
  measureDuration = durationSeconds * 1000UL; // Converte a duração em segundos para milissegundos.
  secondsLeft = durationSeconds; // Define os segundos restantes para a medição (para exibição).

  sampleCount = 0; // Zera o contador de amostras para a medição.
  velocitySum = 0; // Zera a soma para o cálculo da velocidade RMS.
  aPeak = 0; // Zera o valor de pico de aceleração.
  aRMS = 0; // Zera o valor RMS de aceleração.
  stdDev = 0; // Zera o valor do desvio padrão.
  freqDominant = 0; // Zera a frequência dominante.
  lastSampleTime = 0; // Zera o tempo da última amostra.
  vibroState = VibroState::VIBRO_CONFIG; // Muda o estado do vibrometro para 'VIBRO_CONFIG' (preparação para medição).
  Serial.println("Iniciando medição..."); // Imprime uma mensagem no Serial Monitor.
}
//vibroState = VibroState::VIBRO_MEASURE; // Comentário indicando um possível estado futuro ou transição.
// Atualiza o processo de medição de vibração, coletando dados do acelerômetro e realizando a FFT.
void updateMeasurement() {
  if (!isMeasuring) return; // Se 'isMeasuring' for falsa, sai da função imediatamente.

  unsigned long now = millis(); // Obtém o tempo atual em milissegundos.
  unsigned long elapsed = now - measureStartTime; // Calcula o tempo decorrido desde o início da medição.
  unsigned long remaining = (measureDuration > elapsed) ? (measureDuration - elapsed) : 0; // Calcula o tempo restante da medição.
  secondsLeft = remaining / 1000; // Converte o tempo restante para segundos.

  // Verifica se é hora de coletar uma nova amostra, baseado na 'SAMPLE_RATE' e se o buffer da FFT ainda não está cheio.
  if (micros() - lastSampleTime >= 1000000UL / SAMPLE_RATE && sampleCount < FFT_SIZE) {
    lastSampleTime = micros(); // Atualiza o tempo da última amostra coletada.

    sensors_event_t event; // Cria um objeto 'event' para os dados do sensor.
    accel.getEvent(&event); // Obtém os dados de aceleração do ADXL345.

    float x = event.acceleration.x - offsetX; // Ajusta a leitura X aplicando o offset de calibração.
    float y = event.acceleration.y - offsetY; // Ajusta a leitura Y aplicando o offset de calibração.
    float z = event.acceleration.z - offsetZ; // Ajusta a leitura Z aplicando o offset de calibração.

    float acc = sqrt(x * x + y * y + z * z); // Calcula a magnitude vetorial (aceleração total) a partir das componentes ajustadas.

    accBuffer[sampleCount] = acc; // Armazena a magnitude da aceleração no buffer de dados para a FFT.
    timeBuffer[sampleCount] = 0.0; // Preenche o buffer imaginário com zero, pois a FFT é para dados reais.
    velocitySum += acc / SAMPLE_RATE; // Acumula valores para o cálculo aproximado da velocidade RMS (integração numérica simples).

    if (acc > aPeak) aPeak = acc; // Atualiza o 'aPeak' (pico de aceleração) se a 'acc' atual for maior.
    sampleCount++; // Incrementa o contador de amostras.
  }

  // Se o tempo da medição expirou E o buffer da FFT está completamente preenchido.
  if (elapsed >= measureDuration && sampleCount == FFT_SIZE) {
    isMeasuring = false; // Define 'isMeasuring' como falsa, indicando o fim da medição.

    float sum = 0, sumSq = 0; // Variáveis para somar valores e somas dos quadrados para cálculos estatísticos.
    for (int i = 0; i < FFT_SIZE; i++) { // Itera sobre o buffer de aceleração.
      sum += accBuffer[i]; // Soma todos os valores no buffer.
      sumSq += accBuffer[i] * accBuffer[i]; // Soma o quadrado de todos os valores no buffer.
    }

    float mean = sum / FFT_SIZE; // Calcula a média dos valores de aceleração.
    aRMS = sqrt(sumSq / FFT_SIZE); // Calcula o valor RMS (Root Mean Square) da aceleração.

    float variance = 0; // Variável para o cálculo da variância.
    for (int i = 0; i < FFT_SIZE; i++) { // Itera novamente para calcular a variância.
      variance += pow(accBuffer[i] - mean, 2); // Soma o quadrado da diferença de cada amostra em relação à média.
    }
    stdDev = sqrt(variance / FFT_SIZE); // Calcula o desvio padrão.
    vRMS = velocitySum / FFT_SIZE; // Calcula o valor RMS da velocidade (aproximado).

    // Remoção de offset DC (corrente contínua) dos dados antes da FFT.
    double dcOffset = 0; // Variável para o offset DC.
    for (int i = 0; i < FFT_SIZE; i++) { // Calcula a média dos valores no buffer (para encontrar o offset DC).
      dcOffset += accBuffer[i];
    }
    dcOffset /= FFT_SIZE; // Calcula a média.
    for (int i = 0; i < FFT_SIZE; i++) { // Subtrai o offset DC de cada amostra.
      accBuffer[i] -= dcOffset;
    }

    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Aplica a função de janela de Hamming aos dados para reduzir vazamento espectral na FFT.
    FFT.compute(FFT_FORWARD); // Executa a Transformada Rápida de Fourier nos buffers 'accBuffer' (real) e 'timeBuffer' (imaginário).
    FFT.complexToMagnitude(); // Converte os resultados complexos da FFT em seus valores de magnitude.

    double maxMag = 0; // Variável para a magnitude máxima no espectro de frequência.
    int index = 0; // Variável para o índice da frequência dominante.
    for (int i = 1; i < FFT_SIZE / 2; i++) { // Itera sobre a primeira metade do espectro (até a frequência de Nyquist).
      if (accBuffer[i] > maxMag && accBuffer[i] > 5.0) { // Encontra a magnitude máxima, aplicando um filtro de ruído (ignora magnitudes menores que 5.0).
        maxMag = accBuffer[i]; // Atualiza a magnitude máxima.
        index = i; // Armazena o índice da frequência.
      }
    }

    freqDominant = (index * SAMPLE_RATE) / (float)FFT_SIZE; // Calcula a frequência dominante em Hertz com base no índice e na taxa de amostragem/tamanho da FFT.

    Serial.print("Magnitude FFT pico: "); Serial.println(maxMag); // Imprime a magnitude de pico da FFT.

    // Filtro adicional para descartar leituras de frequência dominante em estado estático (baixo movimento/ruído).
    if (aRMS < 0.15 && stdDev < 0.05 && vRMS < 0.01 && maxMag < 5.0) { // Se as métricas de vibração forem muito baixas.
      freqDominant = 0; // Define a frequência dominante como 0, indicando que não há vibração significativa.
    }
    vibroState = VibroState::VIBRO_MEASURE; // Muda o estado do vibrometro para 'VIBRO_MEASURE', que provavelmente exibe os resultados.
    Serial.println("\n--- RESULTADO ---"); // Imprime um cabeçalho para os resultados.
    Serial.print("Pico Acel: "); Serial.print(aPeak, 3); Serial.println(" m/s²"); // Imprime o Pico de Aceleração.
    Serial.print("RMS Acel: "); Serial.print(aRMS, 3); Serial.println(" m/s²"); // Imprime o RMS de Aceleração.
    Serial.print("Desvio padrão: "); Serial.print(stdDev, 3); Serial.println(" m/s²"); // Imprime o Desvio Padrão.
    Serial.print("Velocidade RMS: "); Serial.print(vRMS, 3); Serial.println(" m/s"); // Imprime o RMS de Velocidade.
    Serial.print("Freq. dominante: "); Serial.print(freqDominant, 2); Serial.println(" Hz"); // Imprime a Frequência Dominante.
    Serial.println("-----------------"); // Imprime um rodapé.
  }
}

// Exibe uma imagem monocromática armazenada na memória flash em um display OLED.
void exibirImagemDaFlash(uint32_t enderecoInicial, int largura, int altura, int offsetX, int offsetY) {
  int bytesPorLinha = largura / 8; // Calcula o número de bytes necessários para cada linha da imagem (cada byte representa 8 pixels).
  uint8_t linhaBuffer[bytesPorLinha]; // Cria um buffer temporário para armazenar os dados de uma linha da imagem.

  for (int y = 0; y < altura; y++) { // Itera sobre cada linha da imagem.
    // Lê a linha da imagem diretamente da flash
    uint32_t enderecoLinha = enderecoInicial + (y * bytesPorLinha); // Calcula o endereço de memória flash para o início da linha atual.
    readData(enderecoLinha, linhaBuffer, bytesPorLinha); // Lê os dados da linha da flash para o 'linhaBuffer' (função de "flash.h").

    // Desenha os pixels dessa linha no buffer do display.
    for (int byteIndex = 0; byteIndex < bytesPorLinha; byteIndex++) { // Itera sobre cada byte no buffer da linha.
      uint8_t b = linhaBuffer[byteIndex]; // Pega o byte atual.
      for (int bit = 0; bit < 8; bit++) { // Itera sobre cada bit no byte (representando um pixel).
        int pixelX = offsetX + (byteIndex * 8) + (7 - bit); // Calcula a coordenada X do pixel no display (7 - bit para MSB primeiro).
        int pixelY = offsetY + y; // Calcula a coordenada Y do pixel no display.

        // Verifica se o pixel está dentro dos limites visíveis do display.
        if (pixelX >= 0 && pixelX < 128 && pixelY >= 0 && pixelY < 64) {
          bool isOn = b & (1 << bit); // Verifica se o bit correspondente está ON (1) ou OFF (0).
          display.drawPixel(pixelX, pixelY, isOn ? WHITE : BLACK); // Desenha o pixel no buffer do display: branco se ON, preto se OFF.
        }
      }
    }
  }

  display.display(); // Envia o conteúdo do buffer do display para o hardware do OLED, tornando os pixels visíveis.
}

// Função que verifica se há dados a serem salvos e tenta salvá-los na flash.
bool updateValuesRec(){
  if(saveData){ // Verifica se a flag 'saveData' é verdadeira (indicando que há alterações a serem salvas).
    // Tenta salvar o arquivo de configuração de dados na memória flash.
    if(salvarArquivo(dadosConfig, 0x002000, 0x0020FF)){ // Chama a função 'salvarArquivo' (de "flash.h") para gravar 'dadosConfig' em um endereço específico.
      saveData=false; // Se o salvamento for bem-sucedido, reseta a flag 'saveData' para evitar salvamentos repetidos.
    }
  }
}

// ==== Setup dos botões ====
void setupButtons(){
  pinMode(BUTTON_MENU, INPUT_PULLUP); // Configura o pino BUTTON_MENU como entrada com resistor pull-up interno ativado.
  pinMode(BUTTON_SET, INPUT_PULLUP); // Configura o pino BUTTON_SET como entrada com pull-up.
  pinMode(BUTTON_DOUBLE, INPUT_PULLUP); // Configura o pino BUTTON_DOUBLE como entrada com pull-up.
  pinMode(BUTTON_HALF, INPUT_PULLUP); // Configura o pino BUTTON_HALF como entrada com pull-up.
  pinMode(BUTTON_ENC, INPUT_PULLUP); // Configura o pino BUTTON_ENC como entrada com pull-up.
}

void setup() {
  // Inicializa o pino do LED
  pinMode(LED_PIN, OUTPUT); // Define o pino LED_PIN como uma saída digital.
  digitalWrite(LED_PIN, LOW); // Garante que o LED esteja desligado ao iniciar o sistema.
  // Inicializa o pino do sensor IR
  pinMode(SENSOR_IR_PIN, INPUT); // Define o pino SENSOR_IR_PIN como uma entrada digital.
  setupButtons(); // Chama a função 'setupButtons' para configurar todos os pinos dos botões.

  // Inicia a comunicação I2C com os pinos corretos
  Wire.begin(21, 22); // Inicia a comunicação I2C usando os pinos GPIO 21 (SDA) e 22 (SCL) no ESP32.
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Tenta inicializar o display OLED SSD1306 com o modo de energia e endereço I2C (0x3C é comum).
    Serial.println(F("Falha ao iniciar OLED")); // Se a inicialização falhar, imprime uma mensagem de erro no Serial Monitor.
    while (true); // Trava o programa em um loop infinito se o display não puder ser inicializado, indicando um erro crítico.
  }
  display.clearDisplay(); // Limpa o buffer gráfico do display, garantindo que não haja lixo visual inicial.

  adxlAvailable = accel.begin(); // Tenta iniciar a comunicação com o acelerômetro ADXL345. O retorno indica se foi bem-sucedido.
  if (adxlAvailable) accel.setRange(ADXL345_RANGE_4_G); // Se o acelerômetro estiver disponível, configura sua faixa de medição para +/- 4 G.

  // Inicializa o timer de hardware do ESP32 com resolução de 1us (1MHz)
  timer = timerBegin(0, 80, true); // Inicializa o timer de hardware do ESP32. `0` é o número do timer, `80` é o divisor de frequência (80MHz / 80 = 1MHz, resultando em ticks de 1 microssegundo), `true` indica que o contador é ascendente.
  timerAttachInterrupt(timer, onTimer, true); // Anexa a função 'onTimer' como a rotina de serviço de interrupção (ISR) para este timer. O último 'true' indica que a interrupção será acionada na borda de subida.
  timerAlarm(timer, STB_partTime, true, 0); // Configura o alarme do timer para disparar a cada 'STB_partTime' microssegundos, o alarme se repete (true), e o contador do timer não é zerado após cada disparo (0).

  Serial.begin(115200); // Inicializa a comunicação serial com o computador a 115200 bits por segundo.
  while (!Serial); // Aguarda até que a porta serial esteja aberta (útil para depuração em placas com USB-serial).
  iniciarSPIFlash(); // Chama uma função para inicializar a interface com a memória flash SPI (de "flash.h").
  identificarJEDEC(); // Chama uma função para identificar o fabricante e o tipo da memória flash usando o ID JEDEC (de "flash.h").

  // Exibe imagens de splash screen ou logos da flash na inicialização.
  exibirImagemDaFlash(0x00000, 128, 64, 0, 0); // Carrega e exibe uma imagem completa da flash no display.
  delay(2000); // Aguarda 2 segundos para o usuário ver a imagem.
  display.clearDisplay(); // Limpa o display após o atraso.
  
  exibirImagemDaFlash(0x0041F, 128, 32, 0, 20); // Carrega e exibe uma segunda imagem da flash, possivelmente um logo menor, centralizado verticalmente.

  //carregarArquivo(displayConfig, 0x00680, 0x001AD1); // Linha comentada: uma chamada potencial para carregar um arquivo de configuração de display completo (todos os idiomas).
  carregarArquivo(dadosConfig, 0x002000, 0x0020FF); // Carrega dados de configuração geral ('dadosConfig') de uma área específica da memória flash.
  numIdioma = getValor(dadosConfig, "IDIOMA").toInt(); // Obtém o valor numérico do idioma salvo na configuração e armazena em 'numIdioma'.
  idioma = getSiglaIdioma(numIdioma); // Converte o número do idioma em sua sigla (ex: 1 -> "PT") e armazena em 'idioma'.
  timeMeasure = getValor(dadosConfig, "TIMEMEASURE").toInt(); // Obtém o tempo de duração da medição do vibrometro salvo na configuração.
  timeCalib = getValor(dadosConfig, "TIMECALIB").toInt(); // Obtém o tempo de duração da calibração do vibrometro salvo na configuração.
  delay(1000); // Aguarda 1 segundo.
  carregarArquivoParcial(displayConfig, 0x006DD, 0x001FFF, idioma); // Carrega apenas as partes relevantes do arquivo de configuração de display, filtrando por idioma.
  
  delay(1000); // Aguarda 1 segundo.
  display.clearDisplay(); // Limpa o display novamente.
  /*
  // Seção de código comentada: Este bloco é um exemplo de como você poderia definir valores padrão e salvá-los na flash.
  // Geralmente usado na primeira inicialização ou em um reset de fábrica.
  setValor(dadosConfig, "IDIOMA", "1");
  setValor(dadosConfig, "TIMEMEASURE", "30");
  setValor(dadosConfig, "TIMECALIB", "10");
  setValor(dadosConfig, "FPM", "3000");
  setValor(dadosConfig, "INDICEARQUIVO", "0");
  setValor(dadosConfig, "NEW", "0");
  

  if(salvarArquivo(dadosConfig, 0x002000, 0x0020FF)){
    //Serial.println("Listando dadosConfig:");
    for (auto& c : dadosConfig) {
      Serial.println(c.chave + "=" + c.valor);
    }
  }
  */
}

void loop() {
  handleInput(); // Chama a função para processar as entradas dos botões e do encoder.
  updateValues(); // Chama a função para recalcular os parâmetros de tempo do estroboscópio, lanterna e teste, se necessário.
                  // Comentário original: "Stroboscópio fica com a função rodando, mas não executa os leds. Apenas os contadores do timer." - A lógica aqui apenas prepara os tempos para o timer de hardware.
  updateMeasurement(); // Chama a função para gerenciar o processo de medição do vibrometro.
  updateCalibration(); // Chama a função para gerenciar o processo de calibração do acelerômetro.
  if (inMenu) { // Se o sistema estiver no modo de menu (seleção de modos).
    drawMenu(); // Desenha a interface do menu no display.
    STB_outputEnabled = false; // Desativa a saída do LED do estroboscópio.
    TESTE_calc = false; // Desativa o modo de teste.
    Lant_calc = false; // Desativa o modo de lanterna.
    vibroState = VibroState::VIBRO_HOME; // Reseta o estado do vibrometro para o início.
    updateValuesRec(); // Verifica e salva as configurações na flash, se alguma alteração tiver sido marcada.
  } else { // Se o sistema estiver em um modo de operação específico (fora do menu).
    drawScreen(currentMode); // Desenha a tela correspondente ao modo de operação atualmente selecionado.
  }
  
  if (!inMenu) { // Se o sistema não estiver no menu (ou seja, está em um modo ativo).
    switch (selectedMode) { // Executa a lógica específica para o modo de operação selecionado.
      case Mode::FREQUENCY: { // Lógica para o modo "FREQUENCY" (Estroboscópio).
        STB_outputEnabled = true; // Habilita a saída do LED para o estroboscópio.
        //Comandos para alterar o valor usando o Encoder
        long newPos = encoder.read() / 4; // Lê a posição atual do encoder e divide por 4 para reduzir a sensibilidade.
        if (inSubmenu) { // Se estiver no submenu (modo de ajuste de fase).
          int delta = newPos - STB_lastEncoderPos; // Calcula a diferença na posição do encoder.
          STB_phaseDegrees = constrain(STB_phaseDegrees + delta, 0, 359); // Ajusta os graus de fase, limitando entre 0 e 359.
          STB_lastEncoderPos = newPos; // Atualiza a última posição do encoder lida.
          saveData = setValor(dadosConfig, "FPM", String(fpm)); // Marca os dados de configuração para serem salvos, atualizando o FPM (embora aqui seja a fase, FPM está incluído no contexto).
          STB_calc = true; // Marca para recalcular os tempos do estroboscópio devido à mudança de fase.
        } else if (!inSubmenu) { // Se não estiver no submenu (modo de ajuste de FPM).
          int delta = newPos - STB_lastEncoderPos; // Calcula a diferença na posição do encoder.
          if (inEncoder) { // Se o botão do encoder estiver pressionado (sensibilidade aumentada).
            delta = delta * 10; // Multiplica a mudança por 10.
          }
          fpm = constrain(fpm + delta, minFPM, maxFPM); // Ajusta o FPM, garantindo que esteja dentro dos limites minFPM e maxFPM.
          STB_lastEncoderPos = newPos; // Atualiza a última posição do encoder lida.
          saveData = setValor(dadosConfig, "FPM", String(fpm)); // Marca os dados de configuração para serem salvos, atualizando o FPM.
          STB_calc = true; // Marca para recalcular os tempos do estroboscópio devido à mudança de FPM.
        }
        break; // Sai do 'switch'.
      }
      case Mode::RPM:{ // Lógica para o modo "RPM" (Medição de RPM).
        unsigned long agora = millis(); // Obtém o tempo atual em milissegundos.
        bool estadoAtual = digitalRead(SENSOR_IR_PIN); // Lê o estado atual do pino do sensor IR.
  
        // Detecta pulso (borda de subida: LOW -> HIGH)
        if (estadoAtual == HIGH && estadoAnterior == LOW) { // Se o sensor IR mudou de LOW para HIGH (detectou uma marca).
          contagemPulsos++; // Incrementa o contador de pulsos.
        }
        estadoAnterior = estadoAtual; // Atualiza o estado anterior do sensor IR para a próxima leitura.
        // A cada TEMPO_LEITURA_RPM milissegundos, calcula e imprime o RPM
        if (agora - ultimaLeitura >= TEMPO_LEITURA_RPM) { // Se o tempo definido para a leitura de RPM passou.
          ultimaLeitura = agora; // Atualiza o tempo da última leitura.
          // Se há 1 marca por rotação, então: RPM = pulsos * 60
          int rpm = contagemPulsos * 60; // Calcula o RPM (pulsos por segundo * 60 segundos por minuto).
          rpmValue = rpm; // Armazena o valor de RPM calculado.
          // Reinicia a contagem
          contagemPulsos = 0; // Zera o contador de pulsos para a próxima janela de medição.
        }
        break; // Sai do 'switch'.
      }
      case Mode::LANTERN: // Lógica para o modo "LANTERN" (Lanterna).
        STB_outputEnabled = true; // Habilita a saída do LED para funcionar como uma lanterna.
        Lant_calc = true; // Marca para recalcular os parâmetros da lanterna (garantindo que o LED fique ligado constantemente).
        break; // Sai do 'switch'.
      case Mode::TEST: { // Lógica para o modo "TEST" (Teste de varredura de FPM).
        static unsigned long ultimoTempo = 0; // Variável estática para o tempo do último passo no teste de FPM.
        static float fpmAtual = 30.0; // Variável estática para o FPM atual na varredura do teste.
        static const float passo = 30.0; // Define o incremento do FPM a cada passo do teste.
        static const unsigned long intervaloTeste = 100; // Define o intervalo de tempo em milissegundos entre cada passo de FPM no teste.

        unsigned long agora = millis(); // Obtém o tempo atual em milissegundos.
        if(fpmTest.isRunning()){ // Se o timer de teste de FPM estiver ativo (o teste está em andamento).
          if (agora - ultimoTempo >= intervaloTeste) { // Se o tempo para o próximo passo de FPM expirou.
            TESTE_fpm = fpmAtual; // Define o FPM atual para o modo de teste.
            updateValues(); // Atualiza os parâmetros do estroboscópio com o novo FPM de teste.
            fpmAtual += passo; // Incrementa o FPM para o próximo passo.
            if (fpmAtual >= maxFPM) { // Se o FPM atual atingir ou exceder o limite máximo.
              fpmAtual = 30.0; // Reinicia o ciclo de FPM para começar novamente do mínimo.
            }
            ultimoTempo = agora; // Atualiza o tempo do último passo do teste.
          }
        } else if(fpmTest.isExpired()){ // Se o timer de teste de FPM expirou (o teste foi concluído).
          fpmAtual = 30; // Reseta o FPM atual para o valor inicial.
          STB_outputEnabled = false; // Desativa a saída do LED.
          TESTE_calc = false; // Desativa o modo de teste.
        } else { // Se o timer de teste não está rodando (ainda não foi iniciado ou já foi desativado manualmente).
          STB_outputEnabled = false; // Garante que o LED esteja desligado.
          TESTE_calc = false; // Garante que o modo de teste esteja desativado.
        }
        break; // Sai do 'switch'.
      }
    }
  }
  // nada aqui // Este comentário indica que não há mais lógica de controle de modo fora do 'if (!inMenu)'.
}

// Lida com todas as entradas dos botões e atualiza o estado da interface do usuário (menu, submenus, modos).
void handleInput() {
  static bool lastMenuState = HIGH; // Variável estática para o último estado lido do botão MENU (para debounce).
  static bool lastSetState = HIGH; // Variável estática para o último estado lido do botão SET (para debounce).
  static unsigned long lastDebounceTimeMenu = 0; // Variável estática para o tempo do último debounce válido do botão MENU.
  static unsigned long lastDebounceTimeSet = 0; // Variável estática para o tempo do último debounce válido do botão SET.

  // Processa o botão MENU
  if (checkButtonDebounce(BUTTON_MENU, lastMenuState, lastDebounceTimeMenu)) { // Verifica se o botão MENU foi pressionado com debounce.
    if (inSubmenu) { // Se o usuário estiver atualmente em um submenu.
      // Se estiver no submenu, apenas sai dele
      inSubmenu = false; // Sai do submenu.
    } else if (!inMenu) { // Se o usuário não estiver no menu principal (está em um modo de operação).
      // Se estiver fora do menu, entrar no menu com o modo atual
      currentMode = selectedMode; // Define o modo atual do menu para o modo que estava sendo executado.
      inMenu = true; // Entra no menu principal.
    } else { // Se o usuário já estiver no menu principal.
      currentMode = static_cast<Mode>((static_cast<int>(currentMode) + 1) % static_cast<int>(Mode::NUM_MODES)); // Avança para o próximo modo no menu, ciclando de volta para HOME após o último modo.
    }
  }

  // Processa o botão SET
  if (checkButtonDebounce(BUTTON_SET, lastSetState, lastDebounceTimeSet)) { // Verifica se o botão SET foi pressionado com debounce.
    if (inMenu) { // Se o usuário estiver no menu principal.
      selectedMode = currentMode; // O modo visível ('currentMode') é selecionado para execução.
      inMenu = false; // Sai do menu principal, entrando no modo selecionado.
    } else { // Se o usuário já estiver em um modo de operação.
      if (selectedMode == Mode::VIBROMETER) { // Se o modo selecionado for Vibrometro.
        switch (vibroState) { // Ação do botão SET depende do estado atual do Vibrometro.
          //Clique SET para iniciar
          case VibroState::VIBRO_HOME: // No estado inicial do Vibrometro.
            vibroState = VibroState::VIBRO_CALIB; // Transita para o estado de calibração.
            break; // Sai do 'switch'.
          case VibroState::VIBRO_CALIB: // No estado de calibração.
            startCalibration(timeCalib); // Inicia o processo de calibração com o tempo configurado.
            setValor(dadosConfig, "TIMECALIB", String(timeCalib)); // Salva o tempo de calibração na configuração.
            break; // Sai do 'switch'.
          case VibroState::VIBRO_IDLE: // No estado ocioso após calibração.
            startMeasurement(timeMeasure); // Inicia o processo de medição com o tempo configurado.
            saveData = setValor(dadosConfig, "TIMEMEASURE", String(timeMeasure)); // Marca para salvar o tempo de medição na configuração.
            break; // Sai do 'switch'.
          case VibroState::VIBRO_CONFIG: // No estado de configuração de medição.
            //vibroState = VibroState::VIBRO_MEASURE; // Linha comentada: possível transição para medição ativa.
            //measureSeismicActivity(); // Linha comentada: possível função para iniciar a atividade sísmica (medição).
            break; // Sai do 'switch'.
          case VibroState::VIBRO_MEASURE: // No estado de medição (exibindo resultados em tempo real).
            vibroState = VibroState::VIBRO_RESULT; // Transita para o estado de exibição final de resultados.
            break; // Sai do 'switch'.
          case VibroState::VIBRO_RESULT: // No estado de exibição final de resultados.
            vibroState = VibroState::VIBRO_HOME; // Volta para o estado inicial do Vibrometro.
            break; // Sai do 'switch'.
        }
      } else if (selectedMode == Mode::RPM) { // Se o modo selecionado for RPM.
        inSubmenu = false; // Garante que não esteja em um submenu.
        if(rpmValue >= 30){ // Se o valor de RPM for 30 ou mais (evitando salvar ruído).
          fpm = rpmValue; // Define o FPM do estroboscópio para o valor de RPM medido.
          saveData = setValor(dadosConfig, "FPM", String(fpm)); // Marca para salvar o novo FPM na configuração.
          msgTimer.startTimer(2); // Inicia um timer de 2 segundos para exibir uma mensagem (provavelmente "gravando").
        }
      } else if(selectedMode == Mode::TEST){ // Se o modo selecionado for TEST.
        STB_outputEnabled = true; // Habilita a saída do estroboscópio para o teste.
        TESTE_calc = true; // Marca para iniciar os cálculos do modo de teste.
        fpmTest.startTimer(10); // Inicia o timer para a duração total do teste de FPM (10 segundos).
      }else if(selectedMode == Mode::LANTERN || currentMode == Mode::ABOUT){ // Se o modo selecionado for LANTERN ou ABOUT.
        inSubmenu = false; // Garante que não esteja em um submenu (não há submenus nesses modos).
      } else { // Para outros modos que podem ter um submenu.
        inSubmenu = !inSubmenu; // Alterna o estado do submenu (entra ou sai).
        STB_outputEnabled = false; // Desativa a saída do estroboscópio ao entrar/sair de submenus (para evitar flashes indesejados).
      }
    }
  }

  // Processa o botão DOUBLE
  if (checkButtonDebounce(BUTTON_DOUBLE, lastSetState, lastDebounceTimeSet)) { // Verifica se o botão DOUBLE foi pressionado com debounce.
    if (inMenu) { // Se estiver no menu principal.
      currentMode = static_cast<Mode>((static_cast<int>(currentMode) + 1) % static_cast<int>(Mode::NUM_MODES)); // Avança para o próximo modo no menu, ciclando.
    } else if(selectedMode == Mode::HOME){ // Se o modo selecionado for HOME (tela inicial), usado para rolagem de mensagens.
      if (topLineIndex < totalLines - lineShowMsg) { // Verifica se ainda há linhas para rolar para baixo.
        topLineIndex++; // Rola para a próxima linha superior.
      }
    } else if(currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_CALIB){ // Se estiver no Vibrometro, estado de Calibração.
      timeCalib = constrain(timeCalib - 5, 5, 30); // Diminui o tempo de calibração em 5 segundos, limitando entre 5 e 30.
    } else if(currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_IDLE){ // Se estiver no Vibrometro, estado Ocioso.
      timeMeasure = constrain(timeMeasure - 10, 10, 60); // Diminui o tempo de medição em 10 segundos, limitando entre 10 e 60.
    } else if(currentMode == Mode::CONFIG){ // Se estiver no modo CONFIG.
      numIdioma = (numIdioma - 1 + 1) % 4 + 1; // Avança para o próximo idioma na lista (1, 2, 3, 4, e volta para 1).
      setValor(dadosConfig, "IDIOMA", String(numIdioma)); // Marca para salvar o novo idioma na configuração.
    } else { // Para outros modos.
      if (selectedMode == Mode::FREQUENCY && !inSubmenu) { // Se for o modo FREQUENCY e não estiver no submenu de fase.
        adjustFPM(0.5); // Reduz o FPM pela metade.
        STB_calc = true; // Marca para recalcular os tempos do estroboscópio.
      }
    }
  }

  // Processa o botão HALF
  if (checkButtonDebounce(BUTTON_HALF, lastSetState, lastDebounceTimeSet)) { // Verifica se o botão HALF foi pressionado com debounce.
    if (inMenu) { // Se estiver no menu principal.
      currentMode = static_cast<Mode>((static_cast<int>(currentMode) - 1 + static_cast<int>(Mode::NUM_MODES)) % static_cast<int>(Mode::NUM_MODES)); // Retrocede para o modo anterior no menu, ciclando.
    } else if(selectedMode == Mode::HOME){ // Se o modo selecionado for HOME (tela inicial), usado para rolagem de mensagens.
      if (topLineIndex > 0) { // Verifica se ainda há linhas para rolar para cima.
        topLineIndex--; // Rola para a linha superior anterior.
      }
    } else if(currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_CALIB){ // Se estiver no Vibrometro, estado de Calibração.
      timeCalib = constrain(timeCalib + 5, 5, 30); // Aumenta o tempo de calibração em 5 segundos, limitando entre 5 e 30.
    } else if(currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_IDLE){ // Se estiver no Vibrometro, estado Ocioso.
      timeMeasure = constrain(timeMeasure + 10, 10, 60); // Aumenta o tempo de medição em 10 segundos, limitando entre 10 e 60.
    } else if(currentMode == Mode::CONFIG){ // Se estiver no modo CONFIG.
      numIdioma = (numIdioma - 1 - 1 + 4) % 4 + 1; // Retrocede para o idioma anterior na lista (1, 2, 3, 4, e volta para 4).
      setValor(dadosConfig, "IDIOMA", String(numIdioma)); // Marca para salvar o novo idioma na configuração.
    } else { // Para outros modos.
      if (selectedMode == Mode::FREQUENCY && !inSubmenu) { // Se for o modo FREQUENCY e não estiver no submenu de fase.
        adjustFPM(2.0); // Dobra o FPM.
        STB_calc = true; // Marca para recalcular os tempos do estroboscópio.
      }
    }
  }

  // Processa o botão do ENCODER (clique)
  if (checkButtonDebounce(BUTTON_ENC, lastMenuState, lastDebounceTimeMenu)) { // Verifica se o botão do encoder foi pressionado com debounce.
    inEncoder = !inEncoder; // Alterna o estado de 'inEncoder' (liga/desliga o modo de sensibilidade aumentada do encoder).
    if (inEncoder) { // Se 'inEncoder' for true.
      //FAZ // Comentário para ações a serem realizadas quando o modo de sensibilidade é ativado.
    } else { // Se 'inEncoder' for false.
      //NÃO FAZ // Comentário para ações a serem realizadas quando o modo de sensibilidade é desativado.
    }
  }
}

// ==== Tela do Menu ====
void drawMenu() {
  display.clearDisplay(); // Limpa o buffer gráfico do display.
  display.setTextSize(1); // Define o tamanho do texto para 1 (padrão, pequeno).
  display.setTextColor(SSD1306_WHITE); // Define a cor do texto para branco.
  display.setCursor(0, 0); // Define a posição do cursor no canto superior esquerdo (0,0).
  display.println(getValor(displayConfig, "SELECTMODE"+idioma)); // Imprime a string "SELECTMODE" (do arquivo de configuração) para o idioma atual, seguida de uma nova linha.
  display.setTextSize(2); // Define o tamanho do texto para 2 (maior).
  display.setCursor(0, 20); // Define a posição do cursor para exibir o nome do modo selecionado.
  display.println(getModeName(currentMode)); // Imprime o nome do modo atualmente selecionado no menu.
  display.setTextSize(1); // Define o tamanho do texto de volta para 1.
  display.setCursor(0, 56); // Define a posição do cursor para uma mensagem de navegação na parte inferior.
  display.println(getValor(displayConfig, "MENUPROX", idioma)); // Imprime a string "MENUPROX" (próximo modo do menu) para o idioma atual.
  display.display(); // Atualiza o display físico com o conteúdo do buffer.
}

// ==== Tela de cada Modo ====
void drawScreen(Mode mode) {
  // Limita a taxa de atualização para evitar flickering e consumo de CPU
  static unsigned long lastUpdate = 0; // Variável estática para armazenar o tempo da última atualização do display.
  if (millis() - lastUpdate < 50) return; // Se menos de 50 milissegundos se passaram desde a última atualização, sai da função para limitar a taxa de quadros (aprox. 20 FPS).
  lastUpdate = millis(); // Atualiza o tempo da última atualização para o momento atual.

  display.clearDisplay(); // Limpa o buffer gráfico do display.
  display.setTextSize(1); // Define o tamanho do texto para 1.
  display.setTextColor(SSD1306_WHITE); // Define a cor do texto para branco.
  display.setCursor(0, 0); // Define a posição do cursor no canto superior esquerdo.
  display.println(getModeName(mode)); // Imprime o nome do modo atualmente ativo como título da tela.
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE); // Desenha uma linha horizontal para separar o título do conteúdo.
  display.setCursor(0, 14); // Define a posição do cursor para o início do conteúdo principal da tela.

  if (currentMode == Mode::RPM) { // Se o modo atual for RPM.
    display.print(getValor(displayConfig, "RPM_"+idioma+" ")); // Imprime o texto "RPM " no idioma configurado.
    display.print((int)rpmValue); // Imprime o valor inteiro de RPM medido.
    display.setCursor(0, 26); // Define a posição para a próxima linha de texto.
    display.print(msgTimer.isRunning() ? getValor(displayConfig, "RECRPM", idioma) : getValor(displayConfig, "SETRECRPM", idioma)); // Imprime uma mensagem dinâmica sobre gravar RPM ou configurá-lo.
    display.setCursor(0, 56); // Define a posição para a mensagem de navegação.
    display.println(getValor(displayConfig, "MENUPROX", idioma)); // Imprime a mensagem de "próximo menu".

  } else if (currentMode == Mode::ABOUT) { // Se o modo atual for ABOUT (Sobre).
    display.println("Grupo AlfaS: "); // Imprime o nome do grupo.
    display.println(getValor(displayConfig, "ABOUTP1", idioma)); // Imprime a primeira linha da seção "Sobre".
    display.println(getValor(displayConfig, "ABOUTP2", idioma)); // Imprime a segunda linha.
    display.println(getValor(displayConfig, "ABOUTP3", idioma)); // Imprime a terceira linha.
    display.println(getValor(displayConfig, "ABOUTP4", idioma)); // Imprime a quarta linha.
    exibirImagemDaFlash(0x0041F, 32, 32, 90, 15); // Exibe uma imagem (provavelmente um logo) no canto da tela.
    display.setCursor(0, 56); // Define a posição para a linha do site.
    display.println(getValor(displayConfig, "ABOUTSITE", idioma)); // Imprime o endereço do site.
  } else if (currentMode == Mode::TEST) { // Se o modo atual for TEST.
    display.print(fpmTest.isRunning() ? getValor(displayConfig, "TESTING", idioma) : getValor(displayConfig, "TESTER", idioma)); // Imprime "TESTING" se o teste estiver rodando, ou "TESTER" se estiver parado.
    display.setCursor(0, 56); // Define a posição para a mensagem de controle do teste.
    display.println(fpmTest.isRunning() ? getValor(displayConfig, "TESTWAIT", idioma) : getValor(displayConfig, "TESTSET", idioma)); // Imprime "TESTWAIT" (esperar) se rodando, ou "TESTSET" (iniciar) se parado.
  } else if (currentMode == Mode::LANTERN) { // Se o modo atual for LANTERN.
    display.print(getValor(displayConfig, "LANTERN", idioma)); // Imprime o texto "LANTERN" no idioma configurado.
    display.setCursor(0, 56); // Define a posição para a mensagem de navegação.
    display.println(getValor(displayConfig, "MENULANT", idioma)); // Imprime a mensagem de "menu lanterna".
  } else if (currentMode == Mode::CONFIG) { // Se o modo atual for CONFIG.
    display.println(getValor(displayConfig, "SELECTLANG", idioma)); // Imprime o texto "SELECTLANG" (selecionar idioma).
    display.println(getNomeIdioma(numIdioma)); // Imprime o nome completo do idioma atualmente selecionado.
  } else if (currentMode == Mode::FREQUENCY) { // Se o modo atual for FREQUENCY (Estroboscópio).
    display.setTextSize(2); // Define o tamanho do texto para 2 (maior).
    display.setCursor(0, 14); // Define a posição do cursor.
    display.print((int)fpm); // Imprime o valor inteiro de FPM.
    display.setTextSize(1); // Define o tamanho do texto para 1.
    display.setCursor(70, 22); // Define a posição do cursor para a unidade "FPM".
    display.print(getValor(displayConfig, "FPM", idioma)); // Imprime o texto "FPM".
    display.setCursor(0, 34); // Define a posição do cursor para a frequência em Hz.
    display.print(getValor(displayConfig, "HZ", idioma)+" "); // Imprime o texto "HZ".
    display.print(fpm / 60.0, 2); // Imprime o FPM convertido para Hz com 2 casas decimais.
    display.setCursor(0, 46); // Define a posição do cursor para a fase.
    display.print(getValor(displayConfig, "PHASE", idioma)+" "); // Imprime o texto "PHASE".
    display.print(STB_phaseDegrees); // Imprime o valor da fase em graus.
    display.print((char)247); // Imprime o caractere de grau (ASCII 247).
    if(inSubmenu){ // Se estiver no submenu (ajuste de fase).
      display.setCursor(80, 46); // Define a posição de um indicador de edição.
      display.println("<"); // Imprime o indicador "<".
      display.setCursor(0, 56); // Define a posição para a mensagem de controle.
      display.println(getValor(displayConfig, "PHASEEDIT", idioma)); // Imprime a mensagem "PHASEEDIT".
    }
  } else if (currentMode == Mode::VIBROMETER){ // Se o modo atual for VIBROMETER (Vibrometro).
    if (adxlAvailable){ // Se o acelerômetro ADXL345 estiver disponível.
      switch (vibroState) { // Exibe diferentes informações baseadas no estado atual do vibrometro.
        case VibroState::VIBRO_HOME: // Estado inicial do vibrometro.
          display.println(getValor(displayConfig, "CALIBRATE", idioma)); // Imprime a mensagem "CALIBRATE".
          break;
        case VibroState::VIBRO_CALIB: // Estado de calibração do vibrometro.
          if(!isCalibrating){ // Se a calibração não estiver em andamento (tela de seleção de tempo).
            display.print(getValor(displayConfig, "SELECTTIME", idioma)); // Imprime "SELECTTIME".
            display.print(timeCalib); // Imprime o tempo de calibração configurado.
            display.println("s"); // Imprime a unidade "s" (segundos).
            display.println(""); // Linha em branco.
            display.println(getValor(displayConfig, "SETCALIB", idioma)); // Imprime a mensagem para iniciar a calibração.
          }else{ // Se a calibração estiver em andamento.
            display.println(getValor(displayConfig, "CALIBRATING", idioma)); // Imprime "CALIBRATING".
            display.println(""); // Linha em branco.
            display.print(getValor(displayConfig, "RESTTIME", idioma)); // Imprime "RESTTIME" (tempo restante).
            display.print(secondsLeftCalib); // Imprime os segundos restantes da calibração.
            display.println("s"); // Imprime a unidade "s".
          }
          break;
        case VibroState::VIBRO_IDLE: // Estado ocioso após calibração, aguardando medição.
          display.print(getValor(displayConfig, "SELECTTIME", idioma)); // Imprime "SELECTTIME".
          display.print(timeMeasure); // Imprime o tempo de medição configurado.
          display.println("s"); // Imprime a unidade "s".
          display.println(""); // Linha em branco.
          display.println(getValor(displayConfig, "MSGIDLE1", idioma)); // Imprime a primeira linha de mensagem ociosa.
          display.println(getValor(displayConfig, "MSGIDLE2", idioma)); // Imprime a segunda linha de mensagem ociosa.
        break;
        case VibroState::VIBRO_CONFIG: // Estado de configuração ou contagem regressiva para a medição.
          display.println(getValor(displayConfig, "MEASURE", idioma)); // Imprime a mensagem "MEASURE".
          display.println(""); // Linha em branco.
          display.print(getValor(displayConfig, "RESTTIME", idioma)); // Imprime "RESTTIME".
          display.print(secondsLeft); // Imprime os segundos restantes da medição.
          display.println("s"); // Imprime a unidade "s".
          break;
        case VibroState::VIBRO_MEASURE: // Estado durante a medição ou exibição de resultados intermediários.
          display.print("PA: "); display.print(aPeak, 3); display.println("m/s2"); // Imprime Pico de Aceleração (PA).
          display.print("RMS A: "); display.print(aRMS, 3); display.println("m/s2"); // Imprime RMS de Aceleração (RMS A).
          display.print("DP: "); display.print(stdDev, 3); display.println("m/s2"); // Imprime Desvio Padrão (DP).
          display.print("V RMS: "); display.print(vRMS, 3); display.println("m/s"); // Imprime Velocidade RMS (V RMS).
          display.print("FD: "); display.print(freqDominant, 2); display.println("Hz"); // Imprime Frequência Dominante (FD).
          break;
        case VibroState::VIBRO_RESULT: // Estado de exibição final dos resultados da medição.
          display.println(getValor(displayConfig, "RESULTPA", idioma)); // Imprime o cabeçalho para Pico de Aceleração.
          display.println(getValor(displayConfig, "RESULTRMSA", idioma)); // Imprime o cabeçalho para RMS de Aceleração.
          display.println(getValor(displayConfig, "RESULTDP", idioma)); // Imprime o cabeçalho para Desvio Padrão.
          display.println(getValor(displayConfig, "RESULTVRMS", idioma)); // Imprime o cabeçalho para Velocidade RMS.
          display.println(getValor(displayConfig, "RESULTFD", idioma)); // Imprime o cabeçalho para Frequência Dominante.
          break;
      }
    }else{ // Se o acelerômetro ADXL345 NÃO estiver disponível.
      display.println(getValor(displayConfig, "NOTSENSOR", idioma)); // Imprime a mensagem "NOTSENSOR" (sensor não encontrado).
    }
  }
  display.display(); // Atualiza o display físico com todo o conteúdo desenhado.
}

// ==== Retorna nome do modo atual ====
String getModeName(Mode mode) {
  switch (mode) { // Usa uma estrutura 'switch' para retornar o nome do modo com base no valor da enumeração 'Mode'.
    case Mode::HOME: return "StroboTech"; // Retorna o nome para o modo HOME (tela inicial).
    case Mode::FREQUENCY: return getValor(displayConfig, "TITLEFREQ", idioma); // Retorna o título do modo FREQUENCY, localizado para o idioma atual.
    case Mode::RPM: return getValor(displayConfig, "TITLERPM", idioma); // Retorna o título do modo RPM, localizado.
    case Mode::LANTERN: return getValor(displayConfig, "TITLELANT", idioma); // Retorna o título do modo LANTERN, localizado.
    case Mode::VIBROMETER: return getValor(displayConfig, "TITLEVIBRO", idioma); // Retorna o título do modo VIBROMETER, localizado.
    case Mode::TEST: return getValor(displayConfig, "TITLETEST", idioma); // Retorna o título do modo TEST, localizado.
    case Mode::ABOUT: return getValor(displayConfig, "TITLEABOUT", idioma); // Retorna o título do modo ABOUT, localizado.
    case Mode::CONFIG: return getValor(displayConfig, "TITLECONFIG", idioma); // Retorna o título do modo CONFIG, localizado.
  }
}

// Retorna o nome completo de um idioma com base no seu número de ID.
String getNomeIdioma(int lang) {
  switch (lang) { // Usa uma estrutura 'switch' para retornar o nome completo do idioma.
    case 1: return getValor(displayConfig, "LANGPT", idioma); // Retorna o nome "Português", localizado.
    case 2: return getValor(displayConfig, "LANGEN", idioma); // Retorna o nome "English", localizado.
    case 3: return getValor(displayConfig, "LANGES", idioma); // Retorna o nome "Español", localizado.
    case 4: return getValor(displayConfig, "LANGFR", idioma); // Retorna o nome "Français", localizado.
  }
}

// Retorna a sigla de um idioma com base no seu número de ID.
String getSiglaIdioma(int lng) {
  switch (lng) { // Usa uma estrutura 'switch' para retornar a sigla do idioma.
    case 1: return "PT"; // Retorna a sigla "PT" para Português.
    case 2: return "EN"; // Retorna a sigla "EN" para Inglês.
    case 3: return "ES"; // Retorna a sigla "ES" para Espanhol.
    case 4: return "FR"; // Retorna a sigla "FR" para Francês.
  }
}
