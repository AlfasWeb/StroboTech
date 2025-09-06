#include "flash.h"

void iniciarSPIFlash() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  delay(100);
}

void identificarJEDEC() {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x9F);
  uint8_t mf = SPI.transfer(0x00);
  uint8_t dev = SPI.transfer(0x00);
  uint8_t cap = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);

  Serial.print("JEDEC ID: ");
  Serial.print(mf, HEX); Serial.print(" ");
  Serial.print(dev, HEX); Serial.print(" ");
  Serial.println(cap, HEX);
}

void enableWrite() {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x06);
  digitalWrite(PIN_CS, HIGH);
}

void waitBusy() {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x05);
  while (SPI.transfer(0x00) & 0x01);
  digitalWrite(PIN_CS, HIGH);
}

void eraseSector(uint32_t addr) {
  enableWrite();
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x20);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  digitalWrite(PIN_CS, HIGH);
  waitBusy();
}

void writePage(uint32_t addr, const uint8_t* data, size_t len) {
  while (len > 0) {
    uint32_t pageStart = addr & ~(FLASH_PAGE_SIZE - 1);
    uint32_t pageOffset = addr - pageStart;
    uint32_t spaceInPage = FLASH_PAGE_SIZE - pageOffset;
    size_t toWrite = std::min(len, (size_t)spaceInPage);

    enableWrite();
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(0x02);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >> 8) & 0xFF);
    SPI.transfer(addr & 0xFF);
    for (size_t i = 0; i < toWrite; i++) {
      SPI.transfer(data[i]);
    }
    digitalWrite(PIN_CS, HIGH);
    waitBusy();

    addr += toWrite;
    data += toWrite;
    len -= toWrite;
  }
}

void readData(uint32_t addr, uint8_t* buffer, size_t len) {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x03);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  for (size_t i = 0; i < len; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(PIN_CS, HIGH);
}

void limparFlashParcial(uint32_t addrIni, uint32_t addrFim) {
  if (addrFim <= addrIni) return;
  uint8_t paginaVazia[FLASH_PAGE_SIZE];
  memset(paginaVazia, 0xFF, FLASH_PAGE_SIZE);

  while (addrIni < addrFim) {
    size_t pagina = std::min((size_t)FLASH_PAGE_SIZE, (size_t)(addrFim - addrIni));
    writePage(addrIni, paginaVazia, pagina);
    addrIni += pagina;
  }
}

/*void exibirImagemDaFlash(
  uint32_t enderecoInicial,
  int largura,
  int altura,
  int offsetX,
  int offsetY,
  void (*pixelHandler)(int x, int y, bool on))
  {
  int bytesPorLinha = largura / 8;
  uint8_t linhaBuffer[bytesPorLinha];

  for (int y = 0; y < altura; y++) {
    uint32_t enderecoLinha = enderecoInicial + (y * bytesPorLinha);
    readData(enderecoLinha, linhaBuffer, bytesPorLinha);

    for (int byteIndex = 0; byteIndex < bytesPorLinha; byteIndex++) {
      uint8_t b = linhaBuffer[byteIndex];
      for (int bit = 0; bit < 8; bit++) {
        int pixelX = offsetX + (byteIndex * 8) + (7 - bit);
        int pixelY = offsetY + y;

        bool isOn = b & (1 << bit);
        pixelHandler(pixelX, pixelY, isOn);
      }
    }
  }
}*/
