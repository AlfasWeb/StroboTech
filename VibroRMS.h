#ifndef VibroRMS_h
#define VibroRMS_h

#include <Arduino.h>

class VibroRMS {
  private:
    float sumSq;
    unsigned long count;

  public:
    VibroRMS() {
      reset();
    }

    // Adiciona nova amostra
    void addSample(float value) {
      sumSq += value * value;
      count++;
    }

    // Retorna RMS atual
    float getRMS() {
      if (count == 0) return 0.0;
      return sqrt(sumSq / count);
    }

    // Zera cálculo
    void reset() {
      sumSq = 0;
      count = 0;
    }
};

#endif
