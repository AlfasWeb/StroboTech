# StroboTech - Estroboscópio e Vibrometria com ESP32

## 🎯 Sobre o Projeto

O **StroboTech** é um dispositivo multifuncional portátil, baseado no microcontrolador ESP32, que combina as funcionalidades de um **Estroboscópio Digital** de alta precisão e um **Vibrômetro** para análise de vibrações, incluindo a medição de Aceleração e Velocidade RMS e a identificação de frequência dominante através de Transformada Rápida de Fourier (**FFT**).

Desenvolvido para manutenção preditiva e diagnóstico rápido, o projeto utiliza um display OLED para interface, sensores acelerômetros de alto desempenho (ADXL345 ou LSM6DS) e um LED de alta potência para o estroboscópio.

---

## ✨ Funcionalidades Principais

### 💡 Modo Estroboscópio (Frequency)
* **Controle de Frequência (FPM):** Ajuste de FPM (Flashes por Minuto) na faixa de **30 a 35.000 FPM**.
* **Ajuste de Fase:** Permite o ajuste da fase em graus (0° a 359°) para análise de componentes em movimento.
* **Duty Cycle:** Controle da duração do pulso de luz (Duty Cycle) de **1% a 100%**.
* **Ajuste Rápido:** Funções para multiplicar (dobrar) ou dividir (reduzir pela metade) o FPM rapidamente.

### ⚙️ Modo Vibrometria (Vibrometer)
* **Medição por Eixos e Vetor:** Utiliza acelerômetros (ADXL345 ou LSM6DS).
* **Calibração:** Realiza a remoção do offset (gravidade) com um tempo de calibração configurável (ex: 5s, 10s, 15s).
* **Análise em Tempo e Frequência:**
    * Cálculo de **Aceleração de Pico (`aPeak`)** e **Aceleração RMS (`aRMS`)**.
    * Cálculo da **Velocidade RMS (`vRMS`)**.
    * Cálculo do **Desvio Padrão**.
    * Identificação da **Frequência Dominante** (em Hz) usando **FFT**.
* **Configurações de FFT:** Ajuste do tamanho da amostra (FFT Size - 512, 1024, 2048) e da Taxa de Amostragem (Sample Rate - 100 a 10000 Hz).
* **Função de Janelamento:** Seleção entre as janelas de **Hanning** e **Hamming** para a FFT.

### 🔄 Modo Medição de RPM (RPM)
* Mede Rotações Por Minuto (RPM) utilizando um sensor infravermelho (IR).
* Permite a calibração com um Fator de Calibração ajustável (`CalibFat`).
* O valor de RPM medido pode ser transferido para o FPM do estroboscópio para sincronização.

### 🔦 Outras Funções
* **Lanterna:** Acende o LED de alta potência fixamente.
* **Teste:** Modo de teste de varredura automática de FPM.
* **Over-The-Air (OTA):** Atualização de firmware e gerenciamento de arquivos CSV via Wi-Fi Access Point (AP).
* **Gerenciamento de Dados:** Download e exclusão de arquivos de medição CSV através do servidor Web OTA.
* **Configuração de Idioma:** Suporte a múltiplos idiomas (PT, EN, ES, FR).
* **Reset de Fábrica:** Opção para reverter as configurações salvas na EEPROM para os valores padrão.

---

## 🛠️ Hardware Necessário

| Componente | Uso Principal | Pinos (ESP32) | Notas |
| :--- | :--- | :--- | :--- |
| **Microcontrolador** | ESP32 | - | Placa de desenvolvimento. |
| **Display** | Adafruit SSD1306 (OLED 128x64) | SDA (21), SCL (22) | Interface de usuário. |
| **Acelerômetros** | Adafruit ADXL345 / Adafruit LSM6DS | I2C | Vibrometria e calibração. |
| **LED** | LED de Alta Potência | LED_PIN (2) | Estroboscópio / Lanterna. |
| **Sensor RPM** | Sensor Infravermelho (IR) | SENSOR_IR_PIN (4) | Medição de RPM. |
| **Encoder Rotativo** | Navegação no Menu e Ajustes | ENCODER_PIN_A (33), ENCODER_PIN_B (32) | Controle de FPM, Fase, Menu etc. |
| **Botões** | Menu, Set, Half, Double, Enc | 25, 13, 27, 26, 17 | Interação do usuário. |
| **Power** | Pino ON/OFF | ONOFF (14) | Controle de energia. |

---

## 💻 Estrutura do Código

O projeto está organizado em torno de um *loop* principal que gerencia a entrada de botões (`handleInput`), processa o modo OTA (`processarOTA`) e atualiza a lógica específica de cada modo:

1.  **Estados do Sistema:** A variável `currentMode` (`Mode::HOME`, `Mode::FREQUENCY`, `Mode::VIBROMETER`, etc.) define o comportamento do dispositivo.
2.  **Lógica do Estroboscópio:** Gerenciada principalmente por um *timer* de hardware do ESP32 (`hw_timer_t`) e a função de interrupção `onTimer` para gerar pulsos precisos (microssegundos) no `LED_PIN`.
3.  **Lógica de Vibrometria:** Contém as funções `startCalibration()`, `updateCalibration()`, `startMeasurement()`, e `updateMeasurement()` para controle do ciclo de coleta de dados, cálculo de offsets, integração para velocidade e análise de FFT.
4.  **Armazenamento:** Utiliza EEPROM para salvar configurações persistentes (FPM, Duty Cycle, Sample Rate, etc.).
5.  **Interface:** As funções `drawMenu()` e `drawScreen(Mode mode)` são responsáveis por desenhar o conteúdo na tela OLED.

---

## 🚀 Como Compilar e Rodar

1.  **Ambiente de Desenvolvimento:** Recomenda-se a utilização do **Arduino IDE** ou **VS Code com PlatformIO**.
2.  **Instalação de Bibliotecas:** O projeto requer as seguintes bibliotecas:
    * `Wire.h` (Padrão)
    * `Adafruit_GFX.h`
    * `Adafruit_SSD1306.h`
    * `Adafruit_Sensor.h`
    * `Adafruit_ADXL345_U.h`
    * `Adafruit_LSM6DS.h`
    * `Encoder.h` (Importante: a versão com `#define ENCODER_DO_NOT_USE_INTERRUPTS`)
    * `arduinoFFT.h`
    * `WiFi.h`, `WebServer.h`, `DNSServer.h`, `Update.h` (Para OTA)
    * Arquivos de *header* customizados: `flash.h`, `config.h`, `spiffsConfig.h`, `VibroRMS.h`.
3.  **Configuração do ESP32:** Certifique-se de que a placa ESP32 (ex: ESP32 Dev Module) esteja selecionada e que os pinos I2C (SDA=21, SCL=22) e demais GPIOs estejam conectados conforme a tabela de hardware.
4.  **Upload:** Faça o upload do código `SroboTech.ino` para o ESP32.

---

## 📄 Informações de Versão

* **Versão do Projeto:** V1.0
* **Grupo:** Grupo Alfas - 2025

*Agradecimentos a todos os professores do SENAI que incentivaram e orientaram o projeto.*
