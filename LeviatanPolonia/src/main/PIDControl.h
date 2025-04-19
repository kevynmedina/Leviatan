#ifndef PIDCONTROL_H
#define PIDCONTROL_H

class PIDControl {
  private:
    float kp, ki, kd;
    float errorAnterior;
    float integral;

  public:
    PIDControl(float kp_, float ki_, float kd_)
      : kp(kp_), ki(ki_), kd(kd_), errorAnterior(0), integral(0) {}

    float calcular(float setpoint, float medicion) {
      float error = setpoint - medicion;
      integral += error;
      float derivada = error - errorAnterior;
      errorAnterior = error;
      return kp * error + ki * integral + kd * derivada;
    }

    void reiniciar() {
      errorAnterior = 0;
      integral = 0;
    }
};

#endif
