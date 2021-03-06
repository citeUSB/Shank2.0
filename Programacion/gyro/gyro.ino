/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low gyro data registers.
They can be converted to units of dps (degrees per second) using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).

Example: An L3GD20H gives a gyro X axis reading of 345 with its
default full scale setting of +/- 245 dps. The So specification
in the L3GD20H datasheet (page 10) states a conversion factor of 8.75
mdps/LSB (least significant bit) at this FS setting, so the raw
reading of 345 corresponds to 345 * 8.75 = 3020 mdps = 3.02 dps.
*/
#include <Wire.h>
#include <L3G.h>

int LED1 = PA_2;
L3G gyro;
int sampleNum = 500;
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
  Gyro();
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
  //delay(500);
  while(!gyro.init());
  //digitalWrite(LED1,HIGH);
  //delay(500);
  //digitalWrite(LED1,LOW);
  gyro.enableDefault();
  //Calculate initial DC offset and noise level of gyro
  for(int n=0;n<sampleNum;n++){
    gyro.read();
    dc_offset+=(int)gyro.g.x;
  }
  Serial.write('2');
  dc_offset=dc_offset/sampleNum;
  
  for(int n=0;n<sampleNum;n++){
   gyro.read();
   if((int)gyro.g.x-dc_offset>noise)
     noise=(int)gyro.g.x-dc_offset;
   else if((int)gyro.g.x-dc_offset<-noise)
     noise=-(int)gyro.g.x-dc_offset;
  }
  noise=noise/100; //gyro returns hundredths of degrees/sec
}

void leergiro(){
  sampleTime = millis() - time;
 
  time = millis(); 
  gyro.read();
  rate=((int)gyro.g.x-dc_offset)/100;

  if(rate >= noise || rate <= -noise) angle += ((double)(rate) * sampleTime) / 2000;
  //prev_rate = rate;
  // Keep our angle between 0-359 degrees
  if (angle < 0)  angle += 360;
  else if (angle >= 360) angle -= 360;
  if (angle >=90) digitalWrite(LED1,HIGH);
  else digitalWrite(LED1,LOW);

  send_serial(angle);
}
