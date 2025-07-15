#include "config.h"
#include "flash.h"

bool carregarArquivo(std::vector<Config>& destino, uint32_t addrIni, uint32_t addrFim) {
  destino.clear();
  String total = "";

  const size_t bloco = 512;
  uint8_t buffer[bloco];

  for (uint32_t addr = addrIni; addr < addrFim; addr += bloco) {
    size_t len = (addr + bloco > addrFim) ? (addrFim - addr) : bloco;
    readData(addr, buffer, len);
    for (size_t i = 0; i < len; i++) {
      char c = (char)buffer[i];
      if (c == '\0' || c == (char)0xFF) continue;
      total += c;
    }
  }

  int start = 0;
  while (true) {
    int end = total.indexOf('\n', start);
    if (end == -1) break;
    String linha = total.substring(start, end);
    linha.trim();
    if (linha.length() > 0 && !linha.startsWith("#")) {
      int sep = linha.indexOf('=');
      if (sep != -1) {
        Config cfg;
        cfg.chave = linha.substring(0, sep);
        cfg.valor = linha.substring(sep + 1);
        cfg.chave.trim();
        cfg.valor.trim();
        destino.push_back(cfg);
        delay(5);
      }
    }
    start = end + 1;
  }
  return true;
}

void carregarArquivoParcial(std::vector<Config>& destino, uint32_t addrIni, uint32_t addrFim, const String& idioma) {
  destino.clear();
  String total = "";

  const size_t bloco = 512;
  uint8_t buffer[bloco];

  for (uint32_t addr = addrIni; addr < addrFim; addr += bloco) {
    size_t len = (addr + bloco > addrFim) ? (addrFim - addr) : bloco;
    readData(addr, buffer, len);
    for (size_t i = 0; i < len; i++) {
      char c = (char)buffer[i];
      if (c == '\0' || c == (char)0xFF) continue;
      total += c;
    }
  }

  int start = 0;
  while (true) {
    int end = total.indexOf('\n', start);
    if (end == -1) break;
    String linha = total.substring(start, end);
    linha.trim();

    if (linha.length() > 0 && !linha.startsWith("#")) {
      int sep = linha.indexOf('=');
      if (sep != -1) {
        String chave = linha.substring(0, sep);
        String valor = linha.substring(sep + 1);
        chave.trim();
        valor.trim();

        // ✅ Filtro por idioma (somente se idioma não estiver vazio)
        if (idioma != "") {
          String sufixo = "_" + idioma;
          if (!chave.endsWith(sufixo)) {
            start = end + 1;
            continue;  // Ignora se não for do idioma desejado
          }
        }

        destino.push_back({ chave, valor });
        delay(5);
      }
    }
    start = end + 1;
  }
}

bool salvarArquivo(const std::vector<Config>& lista, uint32_t addrIni, uint32_t addrFim) {
  if (addrFim <= addrIni) return false;

  String conteudo = "";
  for (const auto& c : lista) {
    conteudo += c.chave + "=" + c.valor + "\n";
  }

  const char* data = conteudo.c_str();
  size_t len = conteudo.length();
  size_t totalLen = addrFim - addrIni;

  if (len > totalLen) {
    Serial.println("❌ Dados excedem o espaço disponível.");
    return false;
  }

  //limparFlashParcial(addrIni, addrFim);
  eraseSector(addrIni);

  uint8_t buffer[totalLen];
  memset(buffer, 0xFF, totalLen);
  memcpy(buffer, data, len);

  uint32_t addr = addrIni;
  size_t escrito = 0;
  while (escrito < totalLen) {
    size_t pagina = std::min((size_t)FLASH_PAGE_SIZE, totalLen - escrito);
    writePage(addr, buffer + escrito, pagina);
    addr += pagina;
    escrito += pagina;
  }

  //Serial.println("✅ Dados sobrescritos com preenchimento de 0xFF.");
  return true;
}

String getValor(const std::vector<Config>& lista, const String& chaveProcurada, const String& idioma) {
  String newChave = (idioma != "") ? chaveProcurada + "_" + idioma : chaveProcurada;
  newChave.trim();
  for (const auto& c : lista) {
    if (c.chave == newChave) {
      return c.valor;
    }
  }

  for (const auto& c : lista) {
    if (c.chave == newChave+ "_PT") {
      return c.valor;
    }
  }

  return "";
}

bool setValor(std::vector<Config>& lista, const String& chave, const String& valor, const String& idioma) {
  String newChave = (idioma != "") ? chave + "_" + idioma : chave;
  for (auto& c : lista) {
    if (c.chave == newChave) {
      c.valor = valor;
      return true;
    }
  }
  Config novo;
  novo.chave = newChave;
  novo.valor = valor;
  lista.push_back(novo);
  return true;
}

void carregarImagem(std::vector<Config>& destino, uint32_t addrIni, uint32_t addrFim) {
  destino.clear();
  String total = "";

  const size_t bloco = 512;
  uint8_t buffer[bloco];

  for (uint32_t addr = addrIni; addr < addrFim; addr += bloco) {
    size_t len = (addr + bloco > addrFim) ? (addrFim - addr) : bloco;
    readData(addr, buffer, len);
    for (size_t i = 0; i < len; i++) {
      char c = (char)buffer[i];
      if (c == '\0' || c == (char)0xFF) continue;
      total += c;
    }
  }

  int start = 0;
  while (true) {
    int end = total.indexOf('\n', start);
    if (end == -1) break;
    String linha = total.substring(start, end);
    linha.trim();

    if (linha.length() > 0 && !linha.startsWith("#")) {
      int sep = linha.indexOf('=');
      if (sep != -1) {
        Config cfg;
        cfg.chave = linha.substring(0, sep);   // Ex: "20,30"
        cfg.valor = linha.substring(sep + 1);  // Ex: "FF0000"
        cfg.chave.trim();
        cfg.valor.trim();
        destino.push_back(cfg);
        delay(1);
      }
    }
    start = end + 1;
  }
}

