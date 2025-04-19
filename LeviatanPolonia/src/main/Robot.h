#pragma once
#include "Motor1.h"
#include "EncoderMotor.h"
#include "PIDControl.h"
#include "SensorCNY.h"
//#include <BluetoothSerial.h>


class Robot {
private:
    Motor1 &motorIzq;
    Motor1 &motorDer;
    EncoderMotor &encoderIzq;
    EncoderMotor &encoderDer;
    PIDControl pidIzquierda;
    PIDControl pidDerecha;
    PIDControl pidCentrado;
    SensorCNY &sensorIzq;
    SensorCNY &sensorDer;
    SensorCNY &sensorEnf;
    SensorCNY &sensorLadoDer;
    SensorCNY &sensorLadoIzq;
    SensorCNY &sensorEnfIzq;
    SensorCNY &sensorEnfDer;
    int contador=0;
    bool haciaArriba = true;  // Variable global o estática

    const int PIN_LED=5;



    float setpoint;  // Distancia deseada de la pared
   // BluetoothSerial SerialBT;
    const float DIAMETRO_RUEDA = 2.5; // cm
    const float CIRCUNFERENCIA_RUEDA = PI * DIAMETRO_RUEDA; // 7.854 cm
    const float PULSOS_POR_VUELTA = 311.0;
    const float PASOS_POR_CM = PULSOS_POR_VUELTA / CIRCUNFERENCIA_RUEDA; // 39.59
    const float MAX_PULSOS_POR_SEG = 3421.0;

    const int DISTANCIA_SEGURA = 300;   // Valor del sensor donde comenzar a reducir velocidad
    const int DISTANCIA_MINIMA = 700;   // Valor del sensor donde detenerse por completo
    const int VELOCIDAD_MINIMA = 80;    // Velocidad mínima permitida

    const int RANGE = 700;

    const int RANGEB = 750;
    const int SUMA = 1200;

public:
      Robot(Motor1 &motorIzq, Motor1 &motorDer,
          EncoderMotor &encoderIzq, EncoderMotor &encoderDer,
          SensorCNY &sensorIzq, SensorCNY &sensorDer, SensorCNY &sensorEnf,
          SensorCNY &sensorLadoDer, SensorCNY &sensorLadoIzq,
          SensorCNY &sensorEnfIzq, SensorCNY &sensorEnfDer,
          float KpC, float KiC, float KdC,  // PID Centrado
          float KpD, float KiD, float KdD,  // PID Derecha
          float KpI, float KiI, float KdI)  // PID Izquierda
        : motorIzq(motorIzq), motorDer(motorDer),
          encoderIzq(encoderIzq), encoderDer(encoderDer),
          sensorIzq(sensorIzq), sensorDer(sensorDer), sensorEnf(sensorEnf),
          sensorLadoDer(sensorLadoDer), sensorLadoIzq(sensorLadoIzq),
          sensorEnfIzq(sensorEnfIzq), sensorEnfDer(sensorEnfDer),
          pidCentrado(KpC, KiC, KdC),
          pidDerecha(KpD, KiD, KdD),
          pidIzquierda(KpI, KiI, KdI)
    {}


    void begin() {
        motorIzq.begin();
        motorDer.begin();
        encoderIzq.begin();
        encoderDer.begin();
        sensorIzq.begin();
        sensorDer.begin();
        sensorLadoDer.begin();
        sensorLadoIzq.begin();
        sensorEnfIzq.begin();
        sensorEnfDer.begin();

        pinMode(PIN_LED, OUTPUT);  // Inicializa el pin del LED
    digitalWrite(PIN_LED, LOW);  // Apaga el LED por defecto


      //  SerialBT.begin("chiquillo"); // Nombre del dispositivo Bluetooth
    }

    void moverAdelante() {
      int velIzq=255;
        motorIzq.setVelocidad(velIzq);  
        motorDer.setVelocidad(velIzq);
        Serial.println(velIzq); 

        
    }

    void detenerMotores() {
        motorIzq.detener();  // Detener motor izquierdo
        motorDer.detener();  // Detener motor derecho
            delay(10); 
    }

    void vueltaIzquierda() {
        motorIzq.setVelocidad(-80);  // Detener motor izquierdo
        motorDer.setVelocidad(80);  // Detener motor derecho
        //delay(10);
    }

    void vueltaDerecha() {
        motorIzq.setVelocidad(80);  
        motorDer.setVelocidad(-80); 
        //delay(10);
    }

        void seguirParedDerecha(int velocidadBase) {
        // Leer valores de los sensores
        int valorDer = constrain(sensorDer.leerValor(), 0, 2048);
      
        
        // Calcular ajuste PID
        float ajuste = pidDerecha.calcular(550, valorDer);
        
        // Aplicar el ajuste con velocidad base segura
       // int velocidadBase = 0;  // Velocidad base moderada
        
        // Asegurar que las velocidades estén en rango válido (0-255)
        int velIzq = constrain(velocidadBase + ajuste, -255, 255); //-20 a 255
        int velDer = constrain(velocidadBase - ajuste, -255, 255);

//        ajuste
        //map(var,inf,sup,0,255);


    
       // if(valorEnf>1000){
        
        //  detenerMotores();
        //}else{
         motorIzq.setVelocidad(velIzq);
         motorDer.setVelocidad(velDer); 
       
        
     //   }
        
    
    
    }

     void seguirParedIzquierda(int velocidadBase) {
        // Leer valores de los sensores
        int valorIzq = constrain(sensorIzq.leerValor(), 0, 2048);

//        int valorIzq = constrain(sensorIzq.leerValor(), 400, 2048);
//        int valorLadoIzq = constrain(sensorLadoIzq.leerValor(), 500, 800);
 //       int error = (valorLadoIzq + valorIzq)/2;
        
        
        // Calcular ajuste PID
        float ajuste = pidIzquierda.calcular(600, valorIzq);
        
        // Aplicar el ajuste con velocidad base segura
       // int velocidadBase = 0;  // Velocidad base moderada
        
        // Asegurar que las velocidades estén en rango válido (0-255)
        int velIzq = constrain(velocidadBase - ajuste, -255, 255); //-20 a 255
        int velDer = constrain(velocidadBase + ajuste, -255, 255);

//        ajuste
        //map(var,inf,sup,0,255);


    
       // if(valorEnf>1000){
        
        //  detenerMotores();
        //}else{
         motorIzq.setVelocidad(velIzq);
         motorDer.setVelocidad(velDer); 
       
        
     //   }
        
    
    
    }

    void seguirParedCentrado(int velocidadBase) {
//              encoderIzq.actualizarRPM();
//        int rppp=encoderIzq.getRPS();
//        SerialBT.println(rppp);
        // Leer valores de los sensores
        int valorIzq = sensorIzq.leerValor();
        int valorDer = sensorDer.leerValor();
        int valorEnf = sensorEnf.leerValor();

        
        // Calcular error como la diferencia entre sensores
        float error = valorIzq- valorDer;
        
        // Calcular ajuste PID
        float ajuste = pidCentrado.calcular(0, error);
        float ajusteD = pidCentrado.calcular(0,error);
        
        // Aplicar el ajuste con velocidad base segura
       // int velocidadBase = 0;  // Velocidad base moderada
        
        // Asegurar que las velocidades estén en rango válido (0-255)
        int velIzq = constrain(velocidadBase - ajuste, -255, 255); //-20 a 255
        int velDer = constrain(velocidadBase + ajusteD, -255, 255);

//        ajuste
        //map(var,inf,sup,0,255);


    
       // if(valorEnf>1000){
        
        //  detenerMotores();
        //}else{
         motorIzq.setVelocidad(velIzq);
         motorDer.setVelocidad(velDer); 
       
        
     //   }
        
    
    
    }

   int calcularVelocidadAjustada(int velocidadBase, int valorSensor) {
        // Si no detecta pared frontal, mantener velocidad original
        if(valorSensor < DISTANCIA_SEGURA) {
            return velocidadBase;
        }
        
        // Si está muy cerca, detenerse
        if(valorSensor >= DISTANCIA_MINIMA) {
            return 0;
        }
        
        // Reducción proporcional de la velocidad
        float factor = 1.0 - ((float)(valorSensor - DISTANCIA_SEGURA) / 
                            (DISTANCIA_MINIMA - DISTANCIA_SEGURA));
        
        int velocidadAjustada = velocidadBase * (1.0 - factor);
        
        // Asegurar velocidad mínima
        return max(velocidadAjustada, VELOCIDAD_MINIMA);
    }

void manoDerecha(){
   digitalWrite(PIN_LED, LOW);
  int sensor0 = sensorLadoDer.leerValor(); // IR0
   int sensor1 = sensorEnfDer.leerValor(); // IR1
   int sensor2 = sensorDer.leerValor(); // IR2
   int sensor3 = sensorEnf.leerValor(); // IR3
   int sensor4 = sensorIzq.leerValor(); // IR4
   int sensor5 = sensorEnfIzq.leerValor(); // IR5
  int sensor6 = sensorLadoIzq.leerValor();  // IR6
  if(sensor4 > RANGE && sensor2>RANGE && sensor5>RANGE && sensor1>RANGE ){
        
          vueltaIzquierda();
    
  } else {
         seguirParedDerecha(255);
  }
  

//Serial.print("Sensores: ");
//Serial.print(sensor0); Serial.print(" | ");
//Serial.print(sensor1); Serial.print(" | ");
//Serial.print(sensor2); Serial.print(" | ");
//Serial.print(sensor3); Serial.print(" | ");
//Serial.print(sensor4); Serial.print(" | ");
//Serial.print(sensor5); Serial.print(" | ");
//Serial.println(sensor6);


  
  delay(10);
}



void dosManos() {
  if (haciaArriba) {
    manoDerecha();
    contador++;
    if (contador >= 500) {
      haciaArriba = false;
    }
  } else {
    manoIzquierda();
    contador--;
    if (contador <= 0) {
      haciaArriba = true;
    }
  }
}


void manoIzquierda(){
 digitalWrite(PIN_LED, HIGH);
      int sensor0 = sensorLadoDer.leerValor(); // IR0
       int sensor1 = sensorEnfDer.leerValor(); // IR1
       int sensor2 = sensorDer.leerValor(); // IR2
       int sensor3 = sensorEnf.leerValor(); // IR3
       int sensor4 = sensorIzq.leerValor(); // IR4
       int sensor5 = sensorEnfIzq.leerValor(); // IR5
      int sensor6 = sensorLadoIzq.leerValor();  // IR6
      if(sensor2 > RANGEB && sensor4>RANGEB && sensor1 >RANGEB && sensor5>RANGEB){
         vueltaDerecha();
        } else {
        //  Serial.println("if 11"); SerialBT.println("AVANZAR");
         seguirParedIzquierda(255);
        }
    
//        Serial.print("Sensores: ");
//    Serial.print(sensor0); Serial.print(" | ");
//    Serial.print(sensor1); Serial.print(" | ");
//    Serial.print(sensor2); Serial.print(" | ");
//    Serial.print(sensor3); Serial.print(" | ");
//    Serial.print(sensor4); Serial.print(" | ");
//    Serial.print(sensor5); Serial.print(" | ");
//    Serial.println(sensor6);

delay(10);
}

void derechaRueda(){
        motorIzq.setVelocidad(100);  
        motorDer.setVelocidad(-70); 
      //  delay(10);
}

void izquierdaRueda(){
        motorIzq.setVelocidad(-70);  
        motorDer.setVelocidad(100); 
        //delay(10);
}

void avanzarDistancia(float distancia, int mano){ // izq 1 y der 2 
  distancia = distancia * 50;
  float promedio = 0;
  while(promedio<distancia){
    
  
  float valorEncoderIzq = encoderIzq.obtenerDistancia();
  float valorEncoderDer = encoderDer.obtenerDistancia();
  promedio = ( valorEncoderIzq + valorEncoderDer )/2;
 // Serial.println(promedio);
    if(mano==1){
      manoIzquierda();
    }else{
      manoDerecha();
    }
  }
  
}
  void avanzarLaberinto(){
      
      
      avanzarDistancia(16 ,1);
      detenerMotores();
      encoderIzq.resetPulsos();
      encoderDer.resetPulsos();

      avanzarDistancia(3,2);
      detenerMotores();
      encoderIzq.resetPulsos();
      encoderDer.resetPulsos();
      delay(1000);
      
  }
  

};
