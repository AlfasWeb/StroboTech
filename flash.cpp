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
  Serial.print(mf, HEX);
  Serial.print(" ");
  Serial.print(dev, HEX);
  Serial.print(" ");
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
  while (SPI.transfer(0x00) & 0x01)
    ;
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
// --- Estado do log ---
CsvSlot csvSlots[MAX_CSV_FILES];
uint32_t nextFileId = 0;
bool isFlashLogging = false;
int currentFileIndex = -1;
uint32_t currentStartAddr = 0;
uint32_t currentFlashAddr = 0;

void initCsvSlots() {
  Serial.println("\n🔍 Inicializando slots de CSV na flash externa...");
  nextFileId = 0;

  for (int i = 0; i < MAX_CSV_FILES; i++) {
    uint32_t addr = FLASH_START_ADDR + (i * MAX_CSV_SIZE);
    uint8_t firstByte;
    readData(addr, &firstByte, 1);

    csvSlots[i].index = i;
    csvSlots[i].startAddr = addr;
    csvSlots[i].size = 0;
    csvSlots[i].occupied = false;
    csvSlots[i].fileId = 0;

    // Carrega dados existentes da flash
    if (firstByte != 0xFF) {
      csvSlots[i].occupied = true;
      csvSlots[i].size = getCsvFileSize(i);

      // Lê fileId gravado no início do arquivo (primeiros 4 bytes)
      uint32_t fileId = 0;
      readData(addr, (uint8_t*)&fileId, sizeof(uint32_t));
      csvSlots[i].fileId = fileId;

      if (fileId >= nextFileId) nextFileId = fileId + 1;
    }
  }
  Serial.println("✅ Slots CSV inicializados.");
}
bool startCsvLog(int indexFile) {
  if (indexFile < 0) {
    Serial.println("⚠️ Índice inválido!");
    return false;
  }

  // Determina o slot físico (circular)
  int slot = indexFile % MAX_CSV_FILES;
  uint32_t startAddr = FLASH_START_ADDR + (slot * MAX_CSV_SIZE);

  // Apagar o slot se estiver ocupado
  if (arquivoExiste(slot)) {
    Serial.printf("🧹 Slot físico %d ocupado, apagando...\n", slot);
    apagarCsvFile(slot);
  }

  // Atualiza informações do slot
  currentFileIndex = slot;
  currentStartAddr = startAddr;
  currentFlashAddr = startAddr;

  csvSlots[slot].occupied = true;
  csvSlots[slot].fileId = indexFile;  // mantém o número lógico
  csvSlots[slot].size = 0;

  // Grava fileId no início do slot (primeiros 4 bytes)
  writePage(currentFlashAddr, (const uint8_t*)&csvSlots[slot].fileId, sizeof(uint32_t));
  currentFlashAddr += sizeof(uint32_t);

  Serial.printf("📁 Iniciando log em medicao_%d.csv (slot %d, addr 0x%06lX)\n",
                csvSlots[slot].fileId, slot, (unsigned long)startAddr);

  // Escreve cabeçalho CSV
  const char* header = "SampleRate,Tempo (s),Aceleração (m/s²),X,Y,Z\n";
  writePage(currentFlashAddr, (const uint8_t*)header, strlen(header));
  currentFlashAddr += strlen(header);
  csvSlots[slot].size = currentFlashAddr - currentStartAddr;

  isFlashLogging = true;
  return true;
}
void logCsvSample(float sampleRate, float timeSec, float acc, float x, float y, float z) {
  if (!isFlashLogging) return;

  char line[128];
  int len = snprintf(line, sizeof(line),
                     "%.1f,%.3f,%.5f,%.5f,%.5f,%.5f\n",
                     sampleRate, timeSec, acc, x, y, z);

  if (currentFlashAddr + len > currentStartAddr + MAX_CSV_SIZE) {
    Serial.println("⚠️ Espaço insuficiente na flash para este CSV!");
    isFlashLogging = false;
    return;
  }

  writePage(currentFlashAddr, (const uint8_t*)line, len);
  currentFlashAddr += len;
  csvSlots[currentFileIndex].size = currentFlashAddr - currentStartAddr;
}
bool endCsvLog() {
  if (!isFlashLogging) {
    Serial.println("⚠️ Nenhum log ativo para encerrar!");
    return false;
  }

  isFlashLogging = false;
  csvSlots[currentFileIndex].size = currentFlashAddr - currentStartAddr;

  Serial.printf("✅ Log salvo em: medicao_%d.csv (%.1f KB gravados)\n",
                csvSlots[currentFileIndex].fileId,
                (currentFlashAddr - currentStartAddr) / 1024.0);

  return true;
}
bool apagarCsvFile(int indexFile) {
  if (indexFile < 0 || indexFile >= MAX_CSV_FILES) return false;

  uint32_t startAddr = FLASH_START_ADDR + (indexFile * MAX_CSV_SIZE);
  Serial.printf("🧹 Apagando slot físico %d (addr 0x%06lX)\n", indexFile, (unsigned long)startAddr);

  // Apaga setores do slot
  for (uint32_t addr = startAddr; addr < startAddr + MAX_CSV_SIZE; addr += FLASH_SECTOR_SIZE) {
    eraseSector(addr);
  }

  // Atualiza a tabela em RAM
  csvSlots[indexFile].occupied = false;
  csvSlots[indexFile].size = 0;
  csvSlots[indexFile].fileId = 0;

  Serial.println("✅ Slot apagado com sucesso.");
  return true;
}
String listarCsvFiles() {
  Serial.println("\n📂 Arquivos armazenados na flash externa:");
  String json = "[";
  bool first = true;
  bool algum = false;

  for (int i = 0; i < MAX_CSV_FILES; i++) {
    if (csvSlots[i].occupied) {
      // Garante que o tamanho está atualizado
      size_t fileSize = csvSlots[i].size;
      if (fileSize == 0) fileSize = getCsvFileSize(i);

      algum = true;
      if (!first) json += ",";

      // Nome lógico baseado em fileId
      String name = "medicao_" + String(csvSlots[i].fileId) + ".csv";
      json += "{\"name\":\"" + name + "\",\"size\":" + String(fileSize) + "}";

      Serial.printf("  • %s — %.1f KB\n", name.c_str(), fileSize / 1024.0);
      first = false;
    }
  }

  if (!algum) Serial.println("  (nenhum arquivo encontrado)");
  json += "]";
  return json;
}
bool arquivoExiste(int fileId) {
  for (int i = 0; i < MAX_CSV_FILES; i++) {
    if (csvSlots[i].occupied && csvSlots[i].fileId == fileId) {
      return true;
    }
  }
  return false;
}
void downloadCsvFile(int fileId, WiFiClient* client) {
  // Procura o slot correspondente ao fileId
  int slot = -1;
  for (int i = 0; i < MAX_CSV_FILES; i++) {
    if (csvSlots[i].occupied && csvSlots[i].fileId == fileId) {
      slot = i;
      break;
    }
  }

  if (slot == -1) {
    Serial.printf("⚠️ Arquivo medicao_%d.csv não encontrado.\n", fileId);
    return;
  }

  uint32_t startAddr = FLASH_START_ADDR + (slot * MAX_CSV_SIZE);
  uint8_t buffer[256];
  uint32_t addr = startAddr;

  Serial.printf("📥 Iniciando download de medicao_%d.csv (slot %d)\n", fileId, slot);

  while (addr < startAddr + MAX_CSV_SIZE) {
    readData(addr, buffer, sizeof(buffer));

    bool fim = false;
    for (int i = 0; i < sizeof(buffer); i++) {
      if (buffer[i] == 0xFF) {
        fim = true;
        client->write(buffer, i);
        break;
      }
    }

    if (fim) break;
    client->write(buffer, sizeof(buffer));
    addr += sizeof(buffer);
  }

  Serial.printf("📤 Download de medicao_%d.csv concluído.\n", fileId);
}
size_t getCsvFileSize(int fileId) {
  // Procura o slot correspondente ao fileId
  int slot = -1;
  for (int i = 0; i < MAX_CSV_FILES; i++) {
    if (csvSlots[i].occupied && csvSlots[i].fileId == fileId) {
      slot = i;
      break;
    }
  }

  if (slot == -1) return 0; // arquivo não encontrado

  uint32_t startAddr = FLASH_START_ADDR + (slot * MAX_CSV_SIZE);
  uint8_t buffer[256];
  uint32_t addr = startAddr;
  size_t total = 0;

  while (addr < startAddr + MAX_CSV_SIZE) {
    readData(addr, buffer, sizeof(buffer));
    for (size_t i = 0; i < sizeof(buffer); i++) {
      if (buffer[i] == 0xFF) return total;
      total++;
    }
    addr += sizeof(buffer);
  }
  return total;
}




void apagarArquivosAntigos() {
    Serial.println("🧹 Apagando arquivos antigos...");
    for (int i = 0; i < MAX_CSV_FILES; i++) {
        uint32_t startAddr = FLASH_START_ADDR + i * MAX_CSV_SIZE;
        uint8_t firstByte;
        readData(startAddr, &firstByte, 1);

        if (firstByte != 0xFF) {
            Serial.printf("⚠️ Slot físico %d ocupado, apagando...\n", i);
            apagarCsvFile(i);  // apaga pelo slot físico, não pelo fileId
        }
    }
    Serial.println("✅ Todos os slots antigos foram apagados.");
}

// ==========================================================
// 🧭 Inicialização dos slots
// ==========================================================
/*void initCsvSlots() {
  Serial.println("\n🔍 Inicializando slots de CSV na flash externa...");
  for (int i = 0; i < MAX_CSV_FILES; i++) {
    uint32_t addr = FLASH_START_ADDR + (i * MAX_CSV_SIZE);
    uint8_t firstByte;
    readData(addr, &firstByte, 1);

    csvSlots[i].index = i;
    csvSlots[i].startAddr = addr;
    csvSlots[i].fileId = 0;

    if (firstByte != 0xFF) {
      csvSlots[i].occupied = true;
      csvSlots[i].size = getCsvFileSize(i);
    } else {
      csvSlots[i].occupied = false;
      csvSlots[i].size = 0;
    }
  }
  Serial.println("✅ Slots CSV inicializados.");
}*/
// ==========================================================
// 🔎 Obter tamanho de um arquivo
// ==========================================================
/*size_t getCsvFileSize(int index) {
  if (index < 0 || index >= MAX_CSV_FILES) return 0;

  uint32_t startAddr = FLASH_START_ADDR + (index * MAX_CSV_SIZE);
  uint8_t buffer[256];
  uint32_t addr = startAddr;
  size_t total = 0;

  while (addr < startAddr + MAX_CSV_SIZE) {
    readData(addr, buffer, sizeof(buffer));
    for (size_t i = 0; i < sizeof(buffer); i++) {
      if (buffer[i] == 0xFF) return total;
      total++;
    }
    addr += sizeof(buffer);
  }
  return total;
}*/
// ==========================================================
// 📁 Verificar se arquivo existe
// ==========================================================
/*bool arquivoExiste(int indexFile) {
  if (indexFile < 0 || indexFile >= MAX_CSV_FILES) return false;
  uint32_t startAddr = FLASH_START_ADDR + (indexFile * MAX_CSV_SIZE);
  uint8_t firstByte;
  readData(startAddr, &firstByte, 1);
  return (firstByte != 0xFF);
}*/
// ==========================================================
// 🧹 Apagar slot de CSV
// ==========================================================
/*bool apagarCsvFile(int indexFile) {
  if (indexFile < 0 || indexFile >= MAX_CSV_FILES) return false;

  uint32_t startAddr = FLASH_START_ADDR + (indexFile * MAX_CSV_SIZE);
  Serial.printf("🧹 Apagando slot físico %d (addr 0x%06lX)\n", indexFile, (unsigned long)startAddr);

  for (uint32_t addr = startAddr; addr < startAddr + MAX_CSV_SIZE; addr += FLASH_SECTOR_SIZE) {
    eraseSector(addr);
  }

  csvSlots[indexFile].occupied = false;
  csvSlots[indexFile].size = 0;

  Serial.println("✅ Slot apagado com sucesso.");
  return true;
}*/
// ==========================================================
// 📜 Listar arquivos CSV
// ==========================================================
/*String listarCsvFiles() {
  Serial.println("\n📂 Arquivos armazenados na flash externa:");
  String json = "[";
  bool first = true;
  bool algum = false;

  for (int i = 0; i < MAX_CSV_FILES; i++) {
    if (csvSlots[i].occupied) {
      size_t fileSize = csvSlots[i].size;
      if (fileSize == 0) fileSize = getCsvFileSize(i);

      algum = true;
      if (!first) json += ",";
      String name = "medicao_" + String(csvSlots[i].fileId) + ".csv";
      json += "{\"name\":\"" + name + "\",\"size\":" + String(fileSize) + "}";
      Serial.printf("  • %s — %.1f KB\n", name.c_str(), fileSize / 1024.0);
      first = false;
    }
  }

  if (!algum) Serial.println("  (nenhum arquivo encontrado)");
  json += "]";
  return json;
}*/
// ==========================================================
// 🟢 Iniciar novo log CSV
// ==========================================================
/*bool startCsvLog(int indexFile) {
  if (indexFile < 0) {
    Serial.println("⚠️ Índice inválido!");
    return false;
  }

  // Determina o slot físico (circular)
  int slot = indexFile % MAX_CSV_FILES;
  uint32_t startAddr = FLASH_START_ADDR + (slot * MAX_CSV_SIZE);

  // Apagar o slot se estiver ocupado
  if (arquivoExiste(slot)) {
    Serial.printf("🧹 Slot físico %d ocupado, apagando...\n", slot);
    apagarCsvFile(slot);
  }

  // Atualiza informações
  currentFileIndex = slot;
  currentStartAddr = startAddr;
  currentFlashAddr = startAddr;

  csvSlots[slot].occupied = true;
  csvSlots[slot].fileId = indexFile;  // mantém o número lógico
  csvSlots[slot].size = 0;

  Serial.printf("📁 Iniciando log em medicao_%d.csv (slot %d, addr 0x%06lX)\n",
                indexFile, slot, (unsigned long)startAddr);

  // Escreve cabeçalho CSV
  const char* header = "SampleRate,Tempo (s),Aceleração (m/s²),X,Y,Z\n";
  writePage(currentFlashAddr, (const uint8_t*)header, strlen(header));
  currentFlashAddr += strlen(header);
  csvSlots[slot].size = currentFlashAddr - currentStartAddr;

  isFlashLogging = true;
  return true;
}*/
// ==========================================================
// 🧮 Registrar amostra CSV
// ==========================================================
/*void logCsvSample(float sampleRate, float timeSec, float acc, float x, float y, float z) {
  if (!isFlashLogging) return;

  char line[128];
  int len = snprintf(line, sizeof(line),
                     "%.1f,%.3f,%.5f,%.5f,%.5f,%.5f\n",
                     sampleRate, timeSec, acc, x, y, z);

  if (currentFlashAddr + len > currentStartAddr + MAX_CSV_SIZE) {
    Serial.println("⚠️ Espaço insuficiente na flash para este CSV!");
    isFlashLogging = false;
    return;
  }

  writePage(currentFlashAddr, (const uint8_t*)line, len);
  currentFlashAddr += len;
  csvSlots[currentFileIndex].size = currentFlashAddr - currentStartAddr;
}*/
// ==========================================================
// 🏁 Encerrar log CSV
// ==========================================================
/*bool endCsvLog() {
  if (!isFlashLogging) {
    Serial.println("⚠️ Nenhum log ativo para encerrar!");
    return false;
  }

  isFlashLogging = false;
  csvSlots[currentFileIndex].size = currentFlashAddr - currentStartAddr;

  Serial.printf("✅ Log salvo em: medicao_%d.csv (%.1f KB gravados)\n",
                csvSlots[currentFileIndex].fileId,
                (currentFlashAddr - currentStartAddr) / 1024.0);

  return true;
}*/
// ==========================================================
// 📥 Download de arquivo CSV
// ==========================================================
/*void downloadCsvFile(int indexFile, WiFiClient* client) {
  if (indexFile < 0) return;

  int slot = indexFile % MAX_CSV_FILES;
  if (!arquivoExiste(slot)) {
    Serial.printf("⚠️ Arquivo medicao_%d.csv não encontrado (slot %d).\n", indexFile, slot);
    return;
  }

  uint32_t startAddr = FLASH_START_ADDR + (slot * MAX_CSV_SIZE);
  uint8_t buffer[256];
  uint32_t addr = startAddr;

  while (addr < startAddr + MAX_CSV_SIZE) {
    readData(addr, buffer, sizeof(buffer));

    bool fim = false;
    for (int i = 0; i < sizeof(buffer); i++) {
      if (buffer[i] == 0xFF) {
        fim = true;
        client->write(buffer, i);
        break;
      }
    }

    if (fim) break;
    client->write(buffer, sizeof(buffer));
    addr += sizeof(buffer);
  }

  Serial.printf("📤 medicao_%d.csv enviado ao cliente.\n", indexFile);
}*/