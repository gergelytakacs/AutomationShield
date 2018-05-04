
#include <AutomationShield.h>
#include <FloatShield.h>

int dist;
unsigned long int Ts = 100;
bool enable = false;
int y;
float u;
int r;
float Ti,Td,Kp;
float error;
void setup() {
  Serial.begin(115200);
  FloatShield.initialize();
  Sampling.interruptInitialize(Ts*1000);
  Sampling.setInterruptCallback(StepEnable);
  PIDAbs.setKp(0.86);
  PIDAbs.setKi(1.729);
  PIDAbs.setKd(0.1081);
  Ti = PIDAbs.getTi();
  Td = PIDAbs.getTd();
  Kp = PIDAbs.getKp();
}

void loop() 
{
  if(enable)
  {
    Step();
    enable = false;
  }
}

void StepEnable(void)
{
  enable = true;
}

void Step (void)
{
  y = FloatShield.positionPercent();
  r = FloatShield.referencePercent();

  error = y - r;
  u = PIDAbs.compute(error,25,55,0,100);
  FloatShield.ventInPercent(u);
  Serial.print(r);
  Serial.print(", ");
  Serial.print(u);
  Serial.print(", ");
  Serial.println(y);
}

