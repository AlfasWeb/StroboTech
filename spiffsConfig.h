#ifndef SPIFFSCONFIG_H
#define SPIFFSCONFIG_H

#include "FS.h"
#include "SPIFFS.h"

// Limite de arquivos CSV
extern int MAX_CSV_FILES;

// Variáveis globais CSV
extern File csvFile;
extern String currentCsvName;
extern String csvPrefix;
extern String csvExtension;

// Funções SPIFFS
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
bool fileExists(fs::FS &fs, const char *path);
String readFile(fs::FS &fs, const char *path);
bool writeFile(fs::FS &fs, const char *path, const char *message);
bool appendFile(fs::FS &fs, const char *path, const char *message);
bool renameFile(fs::FS &fs, const char *path1, const char *path2);
bool deleteFile(fs::FS &fs, const char *path);
bool testFileIO(fs::FS &fs, const char *path);
bool iniciarSpiffs();
void testSpiffs();

// Funções CSV
bool startCsvLog(int indexFile);
void logCsvSample(float sampleRate,float timeSec, float acc, float x, float y, float z);
bool endCsvLog();

#endif
