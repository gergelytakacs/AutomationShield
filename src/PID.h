#ifndef PID_H_
#define PID_H_

#include "Timer.h"

enum PidForms{
  absolute,
  incremental
};

class PIDClass{
  
  public:

    PIDClass();
     
    float compute(float err);

    void setKp(float Kp);
    void setKi(float Ki);
    void setKd(float Kd);
    void setTi(float Ti);
    void setTd(float Td);

    float getKp();
    float getKi();
    float getKd();
    float getTi();
    float getTd();

    void beginAbsolute();
    void beginIncremental();

    void beginSaturation(float saturationMin,float saturationMax);
    void stopSaturation();
    void beginAntiWindup(float antiWindupMin,float antiWindupMax);
    void stopAntiWindup();
  
  private:

    float computeAbsForm(float Ts);
    float computeIncForm(float Ts);

    float constrainFloat(float x, float min_x, float max_x);

    PidForms pidForm;
    
    bool saturation;
    float saturationMin;
    float saturationMax;
    
    bool antiWindup;
    float antiWindupMin;
    float antiWindupMax;
    
    float eSum;
    float e[3];
    float u[2];

    float Kp;
    float Ki;
    float Kd;
    float Ti;
    float Td;    
};
extern PIDClass PID; // Declare external instance
#endif
