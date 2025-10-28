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

bool salvarArquivo(const std::vector<Config>& lista, uint32_t addrIni) {
  String conteudo = "";
  for (const auto& c : lista) {
    conteudo += c.chave + "=" + c.valor + "\n";
  }

  const char* data = conteudo.c_str();
  size_t len = conteudo.length();
  
  // O tamanho do setor é uma constante na sua biblioteca, geralmente 4096 bytes
  const size_t SECTOR_SIZE = 4096;

  // Verificação de segurança: checa se o conteúdo cabe no setor
  if (len > SECTOR_SIZE) {
    Serial.println("❌ Dados excedem o tamanho do setor.");
    return false;
  }
  
  // Apaga o setor inteiro a partir do endereço inicial
  // A variável 'addrIni' já aponta para o começo do setor
  eraseSector(addrIni);

  // Buffer para o conteúdo, preenchido com FF para apagar o resto do setor
  uint8_t buffer[SECTOR_SIZE];
  memset(buffer, 0xFF, SECTOR_SIZE);
  memcpy(buffer, data, len);

  // Escreve os dados no setor apagado, página por página
  uint32_t addr = addrIni;
  size_t escrito = 0;
  while (escrito < SECTOR_SIZE) {
    size_t pagina = std::min((size_t)FLASH_PAGE_SIZE, SECTOR_SIZE - escrito);
    writePage(addr, buffer + escrito, pagina);
    addr += pagina;
    escrito += pagina;
  }

  Serial.println("✅ Dados sobrescritos com preenchimento de 0xFF.");
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

//EEPROM
// Uma nova função para carregar dados da EEPROM
bool carregarDadosEEPROM(std::vector<Config>& lista) {
    // Endereço de início da EEPROM
    int addr = 0;
    
    // Inicia a EEPROM (caso ainda não tenha sido iniciada)
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Falha ao iniciar EEPROM!");
        return false;
    }
    
    // Leitura dos dados da EEPROM
    String conteudo = "";
    for (int i = 0; i < EEPROM_SIZE; i++) {
        char c = EEPROM.read(addr + i);
        // Pare de ler quando encontrar um caractere nulo ou espaço vazio (0xFF)
        if (c == 0 || c == 255) {
            break;
        }
        conteudo += c;
    }

    // Limpa a lista antes de preencher com os dados
    lista.clear();
    
    // Processa a String e preenche o vetor de Config
    int pos = 0;
    while (pos < conteudo.length()) {
        int newlinePos = conteudo.indexOf('\n', pos);
        if (newlinePos == -1) {
            newlinePos = conteudo.length();
        }
        String linha = conteudo.substring(pos, newlinePos);
        int equalPos = linha.indexOf('=');
        if (equalPos != -1) {
            Config c;
            c.chave = linha.substring(0, equalPos);
            c.valor = linha.substring(equalPos + 1);
            lista.push_back(c);
        }
        pos = newlinePos + 1;
    }

    // A EEPROM.end() pode ser omitida aqui e chamada apenas na função de salvamento.
    // É mais seguro deixar a comunicação aberta se você for ler e escrever em diferentes partes do código.
    // EEPROM.end();

    Serial.println("✅ Dados da EEPROM carregados.");
    return true;
}
// Uma nova função para salvar dados na EEPROM
bool salvarDadosEEPROM(const std::vector<Config>& lista) {
    // Inicia a EEPROM com o tamanho correto
    if (!EEPROM.begin(EEPROM_SIZE)) {
        return false;
    }

    // Constrói a string de dados
    String conteudo = "";
    for (const auto& c : lista) {
        conteudo += c.chave + "=" + c.valor + "\n";
    }

    // Verifica se o conteúdo cabe no buffer da EEPROM
    if (conteudo.length() > EEPROM_SIZE) {
        Serial.println("❌ Dados excedem o espaco da EEPROM.");
        EEPROM.end();
        return false;
    }

    // Escreve os dados no buffer da EEPROM
    for (int i = 0; i < conteudo.length(); i++) {
        EEPROM.write(i, conteudo[i]);
    }
    
    // Opcional: preenche o restante com zeros
    for (int i = conteudo.length(); i < EEPROM_SIZE; i++) {
      EEPROM.write(i, 0);
    }
    
    // --- PASSO CRUCIAL ---
    // Salva as alterações do buffer para a memória flash
    if (EEPROM.commit()) {
        Serial.println("✅ Dados salvos com sucesso na EEPROM.");
        EEPROM.end();
        return true;
    } else {
        Serial.println("❌ Erro ao salvar dados na EEPROM.");
        EEPROM.end();
        return false;
    }
}