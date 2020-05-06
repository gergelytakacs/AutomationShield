//volanie potrebných knižnic 
#include <BlowShield.h>
//toto neviem, ci nam treba... bude treba odladit. :)
#include <Sampling.h>            // Include sampling
int Ts=4000;//časový interval pre časovanie v milisekundách
#define P 1.0                // PID Kp
#define I 4.0               // PID Ti
#define D 0.00001           // PID Td
float Ek=0;//regulačná odchýlka v čase k
float Esum=0;//suma všetkých odchýlok do času k
float Eback=0;//predchádzajúca regulačná odchýlka
float u;//akčná veličina
//koniec casti PID


void setup() {
  BlowShield.begin();               // Define hardware pins
  BlowShield.calibration();         // Calibrates shield 
}

void loop() {
  // put your main code here, to run repeatedly:
  //toto tu ma byt vlastne krokova funkcia v interrupte. 
 Eback=Ek; //uloženie predchádzajúcej reg. odchýlky skôr, než sa prepíše.
  Ek=BlowShield.referenceRead()-BlowShield.sensorRead();//načítanie aktuálnej reg. odchýlky
  Esum=Esum+Ek;//inkrementácia Esum
  u=PID(P,I,D,Ts,Ek,Esum,Eback);//Zavolanie fcie PID() z knižnice.
uWrite(u);//zápis akčnej veličiny na ledku 
//vypísanie hodnoty na potenciometri v % a svetla z LDR v trubičke ako % na škále minimum a maximum:
Serial.print(BlowShield.referenceRead());
Serial.print(",");
Serial.println(BlowShield.sensorRead());
delayMicroseconds(Ts);
}
