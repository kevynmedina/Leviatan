#ifndef SENSORCNY_H
#define SENSORCNY_H

#include <Arduino.h>

class SensorCNY {
  private:
    uint8_t pin;  // Pin al que está conectado el sensor
    int valorAnalogico;  // Almacena el valor leído del ADC

  public:
    // Constructor para inicializar el pin del sensor
    SensorCNY(uint8_t pin_) : pin(pin_) {}

    // Configura el pin para la lectura analógica
    void begin() {
      adcAttachPin(pin);  // Asocia el pin al ADC
    }

    // Lee el valor analógico del sensor (de 0 a 1023)
    int leerValor() {
      valorAnalogico = analogRead(pin);  // Lee el valor analógico
      return valorAnalogico;
    }

    // Determina si el sensor detecta una línea basándose en el umbral
    bool detectarLinea(int umbral) {
      valorAnalogico = leerValor();
      return valorAnalogico < umbral;  // Si el valor es menor que el umbral, hay línea
    }
};

#endif
