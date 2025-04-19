#pragma once
#include <ESP32Encoder.h>

class EncoderMotor {
private:
    ESP32Encoder encoder;
    long lastCount = 0;  // Mantener lastCount privado
    unsigned long lastMillis = 0;  // Añadir lastMillis para medir el tiempo
    float rps = 0;  // Revoluciones por segundo
    float rpm = 0;  // Revoluciones por minuto // RPM = 660 

public:
    EncoderMotor(int pinA, int pinB) {
        encoder.setCount(0);
        encoder.attachHalfQuad(pinA, pinB);
    }

    void begin() {
        encoder.clearCount();
    }

    void actualizarRPM() {
        unsigned long currentMillis = millis();
        if (currentMillis - lastMillis >= 100) { // cada 100ms
            long newCount = encoder.getCount();
            long delta = newCount - lastCount;
            lastCount = newCount;

            float deltaTime = (currentMillis - lastMillis) / 1000.0; // segundos
            lastMillis = currentMillis;

            rps = delta / 311.0 / deltaTime; // 31 pulsos por vuelta (ajusta según tu encoder)
            rpm = rps * 60.0;
        }
    }

        float getRPS() {
        return rpm;
    }

    long getPulsos() {
        return encoder.getCount();
    }

    // Método para obtener el último conteo de pulsos
    long getLastCount() {
        return lastCount;
    }
    void resetPulsos() {
    encoder.setCount(0);
  }

    // Método para obtener las RPM
    float getRPM() {
        return rpm;
    }

    void reiniciar() {
        encoder.clearCount();
        lastCount = 0;
    }
 //float ppr=315,
    float obtenerDistancia(float diametroRuedaMM=26.5,float ppr = 311) { // 26.5 o 23.5
      float circunferencia= PI*diametroRuedaMM;

      float distancia = (getPulsos()*circunferencia)/900;
      
      return distancia; // en mm
    }

    
};
