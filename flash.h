#ifndef FLASH_H
#define FLASH_H

#include <WiFi.h>
#include <Arduino.h>
#include <SPI.h>

#define PIN_CS    5
#define PIN_MOSI  23
#define PIN_MISO  19
#define PIN_SCK   18
/*
| W25Q Pino | Nome  | ESP32 Pino sugerido |
| --------- | ----- | ------------------- |
| 1         | CS    | GPIO 5              |
| 2         | DO    | GPIO 19 (MISO)      |
| 3         | WP    | 3.3V (puxado alto)  |
| 4         | GND   | GND                 |
| 5         | DI    | GPIO 23 (MOSI)      |
| 6         | CLK   | GPIO 18 (SCK)       |
| 7         | HOLD  | 3.3V (puxado alto)  |
| 8         | VCC   | 3.3V                |

gravado de 0x0000 a 0x1FFF = Logo alfas
gravado de 0x2000 a 0x2FFF = Logo Senai
gravado de 0x3000 a 0x3FFF = Trofeu
gravado de 0x5000 a 0x6FFF = Dados
Inicio gravação CSV = 10000

*/
#define FLASH_SECTOR_SIZE    4096
#define FLASH_PAGE_SIZE      256

void iniciarSPIFlash();
void identificarJEDEC();
void enableWrite();
void waitBusy();
void eraseSector(uint32_t addr);
void writePage(uint32_t addr, const uint8_t* data, size_t len);
void readData(uint32_t addr, uint8_t* buffer, size_t len);
void limparFlashParcial(uint32_t addrIni, uint32_t addrFim);

// --- Configuração da área de CSV na flash externa ---

#define MAX_CSV_FILES       10
#define MAX_CSV_SIZE        (100 * 1024)
#define FLASH_START_ADDR    0x10000
#define FLASH_PAGE_SIZE     256
#define FLASH_SECTOR_SIZE   0x1000
#define META_ADDR           0x08000  // setor reservado para metadados

struct CsvSlot {
    bool occupied;
    size_t size;
    int index;
    uint32_t startAddr;
    uint32_t fileId;  // número lógico do arquivo
};

extern uint32_t nextFileId;
extern CsvSlot csvSlots[MAX_CSV_FILES];  // tabela em RAM

void initCsvSlots();
int getIndexFromName(const String& nome);
size_t getCsvFileSize(int index);
bool arquivoExiste(int indexFile);
String listarCsvFiles();
bool apagarCsvFile(int indexFile);

bool startCsvLog(int indexFile);
void logCsvSample(float sampleRate, float timeSec, float acc, float x, float y, float z);
bool endCsvLog();
void downloadCsvFile(int indexFile, WiFiClient* client);
void apagarArquivosAntigos();
#endif