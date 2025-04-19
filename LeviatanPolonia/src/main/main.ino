#include "Motor1.h"
#include "EncoderMotor.h"
#include "PIDControl.h"
#include "SensorCNY.h"
#include "Robot.h"
#include "Display.h"

//#include <BluetoothSerial.h>
// Definir los pines
const int pinMotorIzq1 = 18;
const int pinMotorIzq2 = 4;
const int pinMotorDer1 = 17;
const int pinMotorDer2 = 16;

const int pinEncoderIzqA = 23;
const int pinEncoderIzqB = 19;
const int pinEncoderDerA = 35;
const int pinEncoderDerB = 34;

const int pinSensorIzq = 27;
const int pinSensorDer = 25;
const int pinSensorEnf = 26; 
const int pinSensorLadoDer = 36;
const int pinSensorLadoIzq = 39;
const int pinSensorEnfIzq = 32;
const int pinSensorEnfDer = 33;
const int PUSH = 12;

// Vueltas


// Instanciar componentes
Motor1 motorIzq(pinMotorIzq1, pinMotorIzq2); 
Motor1 motorDer(pinMotorDer1, pinMotorDer2);
EncoderMotor encoderIzq(pinEncoderIzqA, pinEncoderIzqB);
EncoderMotor encoderDer(pinEncoderDerA, pinEncoderDerB);
Display pantalla;




//Constantes PID Centrado
const float Kp = 0.45;  // Ganancia proporcional
const float Ki = 0.000; // Ganancia integral 0.0001
const float Kd = 0.4;  // Ganancia derivativa
  int contador=0;
//Constantes PID Derecha
const float KpD = 0.50;  // Ganancia proporcional 12 pega
const float KiD = 0.000; // Ganancia integral 0.0001
const float KdD = 0.45;  // Ganancia derivativa
//Constantes PID Izquierda
const float KpI = 0.45;  // Ganancia proporcional
const float KiI = 0.000; // Ganancia integral 0.0001
const float KdI = 0.4;  // Ganancia derivativa



  

//kd 50
SensorCNY sensorIzq(pinSensorIzq);
SensorCNY sensorDer(pinSensorDer);
SensorCNY sensorEnf(pinSensorEnf);
SensorCNY sensorLadoDer(pinSensorLadoDer);
SensorCNY sensorLadoIzq(pinSensorLadoIzq);
SensorCNY sensorEnfIzq(pinSensorEnfIzq);
SensorCNY sensorEnfDer(pinSensorEnfDer);
 
Robot robot(
    motorIzq, motorDer, 
    encoderIzq, encoderDer,
    sensorIzq, sensorDer, sensorEnf,
    sensorLadoDer, sensorLadoIzq,
    sensorEnfIzq, sensorEnfDer,
    Kp, Ki, Kd,    // PID Centrado
    KpD, KiD, KdD, // PID Derecha
    KpI, KiI, KdI  // PID Izquierda
);

            

void setup() {
  //Serial.begin(115200);  // Mayor velocidad para depuración
    pinMode(PUSH, INPUT_PULLUP);
   analogReadResolution(11);
  robot.begin();
  
  // Inicializar sensores
  sensorIzq.begin();
  sensorDer.begin();
  sensorEnf.begin();
  pantalla.begin();
  pantalla.mostrarMapa();


  // Esperar un segundo para estabilizar
  while(digitalRead(PUSH)){
  }
  delay(1000);
}

void loop() {
  //robot.moverAdelante();
  //robot.vueltaDerecha();
  //robot.seguirParedCentrado(255);
 robot.seguirParedDerecha(255);
 //robot.seguirParedIzquierda(255);
 //while(digitalRead(PUSH)){
 //  robot.detenerMotores();
 //}
 //robot.dosManos();
//robot.manoDerecha();
//robot.manoIzquierda();
 //robot.avanzarDistancia(1); // si mando 1 luego multiplico por 50 lo que es una casilla
 robot.avanzarLaberinto();

 
 
 delay(10);

 


  
}
