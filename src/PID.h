#ifndef PID_H_
#define PID_H_

#include "Sampling.h"

// Parent class for PIDAbsClass and PIDIncClass
// This class is abstract. Abstract class cannot be instantiated, because include abstract methods without an implementation

class PIDClass{  
  public:
    PIDClass();
	
    float compute(float err); // compute action without restrictions
    float compute(float err,float saturationMin,float saturationMax); //compute an action within the given boundaries

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
  
  protected:
    virtual void loadVariables(float err)=0;   
    virtual void shiftVariables()=0;

    virtual float getU()=0;
    virtual float computeU()=0;
    virtual void setU(float u)=0;

    float constrainFloat(float x, float min_x, float max_x);

    float Kp;
    float Ki;
    float Kd;
    float Ti;
    float Td;

    float Ts;        
};
#endif
