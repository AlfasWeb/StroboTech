#ifndef FLASH_H
#define FLASH_H

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
/*void exibirImagemDaFlash(
  uint32_t enderecoInicial,
  int largura,
  int altura,
  int offsetX,
  int offsetY,
  void (*pixelHandler)(int x, int y, bool on)
);*/

#endif
