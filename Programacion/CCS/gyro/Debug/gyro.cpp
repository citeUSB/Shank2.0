#include "Energia.h"

#line 1 "C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/gyro/gyro.ino"













#include <Wire.h>
#include <L3G.h>

void setup();
void send_serial(uint32_t dato);
void loop();
void Gyro();
void leergiro();

#line 17
int LED1 = PA_2;
L3G gyro;
int sampleNum = 5;
int dc_offset = 0;
double noise = 0;
unsigned long time;
int sampleTime = 10;
int rate;
int prev_rate = 0;
double angle = 0;
double ang = 0;
double anguloT;
int sample;
char buffer[5];

void setup() {
  Serial.begin(115200);
  Serial.write('1');
  pinMode(LED1,OUTPUT);
  Wire.begin();
  Wire.setModule(0);
  
  Serial.write('2');
}

void send_serial(uint32_t dato) {
  int i;
  for (i = 0; i < 4; i++) {
    buffer[i] = dato >> i * 8;
  }
  buffer[4] = 0xFF;
  for (i = 0; i < 5; i++) {
    Serial.write(buffer[i]);
  }
}

void loop() {
  Serial.write('1');
  delay(100);
  leergiro();
}

void Gyro(){
  digitalWrite(LED1,LOW);
  
  while(!gyro.init());
  
  
  
  gyro.enableDefault();
  
  for(int n=0;n<sampleNum;n++){
    gyro.read();
    dc_offset+=(int)gyro.g.x;
  }
  Serial.write('2');
  dc_offset=dc_offset/sampleNum;
  
  for(int n=0;n<sampleNum;n++){
   
   if((int)gyro.g.x-dc_offset>noise)
     noise=(int)gyro.g.x-dc_offset;
   else if((int)gyro.g.x-dc_offset<-noise)
     noise=-(int)gyro.g.x-dc_offset;
  }
  Serial.write('1');
  noise=noise/100; 
}

void leergiro(){
  sampleTime = millis() - time;
 
  time = millis(); 
  gyro.read();
  rate=((int)gyro.g.x-dc_offset)/100;

  if(rate >= noise || rate <= -noise) angle += ((double)(rate) * sampleTime) / 2000;
  
  
  if (angle < 0)  angle += 360;
  else if (angle >= 360) angle -= 360;
  if (angle >=90) digitalWrite(LED1,HIGH);
  else digitalWrite(LED1,LOW);

  send_serial(angle);
}



