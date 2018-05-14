#ifndef PIDABS_H_
#define PIDABS_H_

#include "PID.h"

class PIDAbsClass:public PIDClass{
  
  public:

    PIDAbsClass();
    using PIDClass::compute;	/* By declaring another function named compute in your child class, you are hiding the base class 
								method named compute. You can prevent this by including it explicitly into the child class*/							 
    float compute(float err,float saturationMin,float saturationMax,float antiWindupMin,float antiWindupMax);
    
  private:

    void loadVariables(float err);
    void shiftVariables();
    float getU();
    float computeU();    
    void setU(float u);

    float e[2];
    float u;
    float eSum;
          
};
extern PIDAbsClass PIDAbs; // Declare external instance
#endif
