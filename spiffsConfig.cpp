#include "spiffsConfig.h"

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}
bool fileExists(fs::FS &fs, const char *path) {
  Serial.printf("Verificando se o arquivo existe: %s\r\n", path);

  if (fs.exists(path)) {
    Serial.println("- arquivo encontrado");
    return true;
  } else {
    Serial.println("- arquivo não encontrado");
    return false;
  }
}
String readFile(fs::FS &fs, const char *path) {
  Serial.printf("Lendo arquivo: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- falha ao abrir arquivo para leitura");
    return String(); // retorna string vazia em caso de erro
  }

  String conteudo;
  while (file.available()) {
    conteudo += char(file.read());
  }
  file.close();

  Serial.println("- leitura concluída");
  return conteudo;
}
bool writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Escrevendo arquivo: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- falha ao abrir arquivo para escrita");
    return false;
  }

  bool sucesso = file.print(message);

  if (sucesso) {
    Serial.println("- arquivo escrito com sucesso");
  } else {
    Serial.println("- falha ao escrever no arquivo");
  }

  file.close();
  return sucesso;
}
bool appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Anexando ao arquivo: %s\r\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("- falha ao abrir arquivo para anexar");
    return false;
  }

  bool sucesso = file.print(message);
  if (sucesso) {
    Serial.println("- mensagem anexada com sucesso");
  } else {
    Serial.println("- falha ao anexar mensagem");
  }

  file.close();
  return sucesso;
}
bool renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renomeando arquivo %s para %s\r\n", path1, path2);

  if (fs.rename(path1, path2)) {
    Serial.println("- arquivo renomeado com sucesso");
    return true;
  } else {
    Serial.println("- falha ao renomear arquivo");
    return false;
  }
}
bool deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Excluindo arquivo: %s\r\n", path);

  if (fs.remove(path)) {
    Serial.println("- arquivo excluído com sucesso");
    return true;
  } else {
    Serial.println("- falha ao excluir arquivo");
    return false;
  }
}
bool testFileIO(fs::FS &fs, const char *path) {
  Serial.printf("Testando leitura e escrita em %s\r\n", path);

  static uint8_t buf[512];
  size_t len = 0;
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- falha ao abrir arquivo para escrita");
    return false;
  }

  size_t i;
  Serial.print("- escrevendo");
  uint32_t start = millis();
  for (i = 0; i < 2048; i++) {
    if ((i & 0x001F) == 0x001F) {
      Serial.print(".");
    }
    if (file.write(buf, 512) != 512) {
      Serial.println("\n- erro na escrita de dados");
      file.close();
      return false;
    }
  }
  Serial.println("");
  uint32_t end = millis() - start;
  Serial.printf(" - %u bytes escritos em %lu ms\r\n", 2048 * 512, end);
  file.close();

  file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- falha ao abrir arquivo para leitura");
    return false;
  }

  len = file.size();
  size_t flen = len;
  start = millis();
  i = 0;
  Serial.print("- lendo");
  while (len) {
    size_t toRead = (len > 512) ? 512 : len;
    if (file.read(buf, toRead) != toRead) {
      Serial.println("\n- erro na leitura de dados");
      file.close();
      return false;
    }
    if ((i++ & 0x001F) == 0x001F) {
      Serial.print(".");
    }
    len -= toRead;
  }
  Serial.println("");
  end = millis() - start;
  Serial.printf("- %u bytes lidos em %lu ms\r\n", flen, end);
  file.close();

  Serial.println("✅ Teste de I/O concluído com sucesso!");
  return true;
}
bool iniciarSpiffs() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return false;
  }
  return true;
}
void testSpiffs(){
  listDir(SPIFFS, "/", 0);
  writeFile(SPIFFS, "/hello.txt", "Hello ");
  appendFile(SPIFFS, "/hello.txt", "World!\r\n");
  readFile(SPIFFS, "/hello.txt");
  renameFile(SPIFFS, "/hello.txt", "/foo.txt");
  readFile(SPIFFS, "/foo.txt");
  deleteFile(SPIFFS, "/foo.txt");
  testFileIO(SPIFFS, "/test.txt");
  deleteFile(SPIFFS, "/test.txt");
  Serial.println("Test complete");
}
