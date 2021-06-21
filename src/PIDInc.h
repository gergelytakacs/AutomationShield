#ifndef PIDINC_H
#define PIDINC_H

#include "PID.h"

class PIDIncClass:public PIDClass{  
  public:
    PIDIncClass();    
  private:
    void loadVariables(float err);
    void shiftVariables();
    float getU();   
    float computeU();
    void setU(float u);
    float e[3];
    float u[2];          
};
extern PIDIncClass PIDInc; // Declare external instance
#endif
