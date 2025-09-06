#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <vector>
#include <EEPROM.h>

struct Config {
  String chave;
  String valor;
};

bool carregarArquivo(std::vector<Config>& destino, uint32_t addrIni, uint32_t addrFim);
bool salvarArquivo(const std::vector<Config>& lista, uint32_t addrIni);
String getValor(const std::vector<Config>& lista, const String& chave, const String& idioma = "");
bool setValor(std::vector<Config>& lista, const String& chave, const String& valor, const String& idioma = "");
void carregarArquivoParcial(std::vector<Config>& destino, uint32_t addrIni, uint32_t addrFim, const String& idioma = "");
void carregarImagem(std::vector<Config>& destino, uint32_t addrIni, uint32_t addrFim);

//EEPROM
#define EEPROM_SIZE 512

bool carregarDadosEEPROM(std::vector<Config>& lista);
bool salvarDadosEEPROM(const std::vector<Config>& lista) ;

#endif
