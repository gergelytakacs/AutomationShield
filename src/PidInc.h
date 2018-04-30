#ifndef PIDINC_H_
#define PIDINC_H_

#include "Pid.h"

class PidIncClass:public PidClass{
  
  public:

    PidIncClass();
    
  private:

    void loadVariables(float err);
    void shiftVariables();
    float getU();   
    float computeU();
    void setU(float u);

    float e[3];
    float u[2];
          
};
extern PidIncClass PidInc; // Declare external instance
#endif
