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
    float compute(float err,float saturationMin,float saturationMax);
    float compute(float err,float saturationMin,float saturationMax,float antiWindupMin,float antiWindupMax);

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
  
  private:

    float computeAbsForm();
    
    float computeIncForm();

    void loadVariables(float err);
    
    float computeU();
    
    void shiftVariables();

    float constrainFloat(float x, float min_x, float max_x);

    PidForms pidForm;
    
    float eSum;
    float e[3];
    float u[2];

    float Kp;
    float Ki;
    float Kd;
    float Ti;
    float Td;

    float Ts;        
};
extern PIDClass PID; // Declare external instance
#endif
