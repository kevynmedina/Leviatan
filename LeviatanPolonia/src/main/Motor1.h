#ifndef MOTOR1_H
#define MOTOR1_H

#include <Arduino.h>

//const int pinMotorIzq1 = 18;
//const int pinMotorIzq2 = 4;
//const int pinMotorDer1 = 17;
//const int pinMotorDer2 = 16;

class Motor1 {
  private:
    int pin1;
    int pin2;

  public:
    Motor1(int pin1_, int pin2_) {
      pin1 = pin1_;
      pin2 = pin2_;
    }

    void begin() {
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
    }

    void setVelocidad(int velocidad) {

      if (velocidad >= 0) {
        analogWrite(pin2,velocidad);
        analogWrite(pin1,0);
      } else {
        analogWrite(pin1, -velocidad);
        analogWrite(pin2,0);
      }
    }

    void detener(){
        analogWrite(pin2, 255);
        analogWrite(pin1, 255);
        
    }

    void setVelocidadC(int velocidad) {
      if (velocidad >= 0) {
        analogWrite(pin2,velocidad);
        analogWrite(pin1,0);
      } else {
        analogWrite(pin1, -velocidad);
        analogWrite(pin2,0);
      }
    }

//    void setVelocidadC(int velocidad) {
//      int pwm = constrain(abs(velocidad), 0, 255);
//      pwm = 255 - pwm; // invertir valor
//      if (velocidad >= 0) {
//        digitalWrite(pinDIR, HIGH);
//        analogWrite(pinPWM, pwm);
//      } else {
//        digitalWrite(pinDIR, LOW);
//        analogWrite(pinPWM, pwm);
//      }
//    }

//    void detener() {
//      analogWrite(pinPWM, 0);
//    }
};

#endif
