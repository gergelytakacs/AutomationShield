#include <MagnetoShield.h>
#include <Sampling.h>
// Nastavenie pre Arduino UNO/DUE - ovela rychlejsie vzorkovanie
#define KP 5.2        //5.2  -  mensie kmitanie, KP viac, TI menej
#define TI 0.3        //0.3
#define TD 0.01       //0.01

bool enable=false;            // premenna pre interrupt
unsigned long Ts=1000;        // perioda vzorkovania [us] pre UNO 2000[us] / pre DUE 650[us]
int T = 3000;                 // perioda zmeny pozicie [ms]
int r[] = {14.5,13.5,15.5,16,14};                   // meraci set 1
//int r[] = {16,16.5,13,12.5,14};                   // meraci set 2
int re = r[0];
int k = 0;
int cl = 0;
int r_size = sizeof(r)/sizeof(r[0]);

// premenne na overenie dlzky vzorkovacej periody
unsigned long Start;
unsigned long Stop;

float y,u,V,I;


void setup() {
  Serial.begin(115200);
  MagnetoShield.begin();               // Nacitaj kniznicu MagnetoShield
  MagnetoShield.calibration();         // Kalibracia

  Sampling.period(Ts);    // Nastavenie vzorkovania periody
  Sampling.interrupt(stepEnable); // Nastavenie interruptu

  // nastavenie diskretnych PID konstant
  PIDAbs.setKp(KP); // Propocna
  PIDAbs.setTi(TI); // Integracna
  PIDAbs.setTd(TD); // Derivacna
  PIDAbs.setTs(Sampling.samplingPeriod); // vzorkovanie

  Serial.print("MIN:");
  Serial.println(MagnetoShield.getMinCalibrated());
    Serial.print("MAX:");
  Serial.println(MagnetoShield.getMaxCalibrated());
  // overenie vzorkovania
  Start = micros();
  step();
  Stop = micros();
  Serial.print("minimalna dlzka periaody: ");
  Serial.println(Stop-Start);
}

void loop() {
  if (cl>=T){
    k++;
    re = r[k];
    cl=0;
  }
  if (k>r_size){
    k=0;
  }
 if (enable) {        // overenie casu
    step();           // vykonanie algoritmu
    cl++;
    enable=false;     // cakanie na dalsiu periodu
  }
}

void stepEnable(){    // definovanie interruptu
  enable=true;        // povolenie dalsej vyorlky
}

void step(){
  y = MagnetoShield.sensorRead();   // poloha magnetu [mm]
  u = PIDAbs.compute((y-re),0,MagnetoShield.getVoltageRef(),-10.6,10.6);   // PID algoritmus   
  MagnetoShield.actuatorWrite(u);   // signal na akcny clen [V]
  V = MagnetoShield.auxReadVoltage();
  I = MagnetoShield.auxReadCurrent();
  Serial.print(re);                          // ziadana hodnota [mm]
  Serial.print(", ");            
  Serial.print(y);                          // aktualna hodnota [mm]
  Serial.print(", ");
  Serial.print(u);                        // vstup na akcny clen [%]
  Serial.print(", ");
  Serial.print(V);                        // Napatie
  Serial.print(", ");
  Serial.println(I);                        // Prud
}

