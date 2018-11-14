#include "Energia.h"

#line 1 "C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/shank/shank.ino"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/qei.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include <Wire.h>
#include <L3G.h>


#define LED PB_0
#define Dist_casilla 168
#define Dist_casilla_A 186 

void setup();
void loop();
void ledsConfig();
void gyroConfig();
void encodersConfig();
void encSumaA();
int medir(int n);
int selectorDato(int selector);
void leergyro();
void PWM_D(int velocidadD, int sentidoD);
void PWM_I(int velocidadI, int sentidoI);
void motorConfig();
void GEN_PWM(int GEN, int OUT,int OUT_BITS, int vel);
void enviarSerial(int dato, int serial);

#line 19
float mmxticks = 8.41;
int estado = 1;
int pwm = 20;
char buffer[9];
int serial;  
int prueba_leds;
int RESET = 0;    
int i;
int N;
int LED1 = PA_2;
int LED2 = PA_3;
int inicio=0;
int selector = 89;
int SW1 = PA_4;
int SW2 = PA_5;


int emisor1=PB_4;
int emisor3=PD_6;
int emisor4=PD_0;
int emisor6=PE_4;


int receptor1=PD_1;
int receptor3=PD_3;
int receptor4=PE_2;
int receptor6=PE_0;

int calibracionFD= 1569;         
int calibracionLD = 1248;         
int calibracionLI = 1327;         
int calibracionFI= 1230;         
int calibracionF=0;          


#define encA PC_6                            
#define encB PC_5                           
volatile long ticksEncA = 0;                
int ticksEncB = 0;                



int Pared_D;
int Pared_I;
int Pared_De;
int Pared_DiD;
int Pared_DiI;


L3G gyro;
int sampleNum = 5;
int dc_offset = 0;
double noise = 0;
unsigned long time;
int sampleTime = 10;
int rate;
int prev_rate = 0;
double angle = 0;
int temp;


#define GPIO_PF2_M1PWM6         0x00050805
#define GPIO_PF3_M1PWM7         0x00050C05
#define GPIO_PA6_M1PWM2         0x00001805
#define GPIO_PA7_M1PWM3         0x00001C05
#define NSLEEP PC_7
#define FAUTL PD_7
#define FORWARD 1
#define BACKWARD 0




void setup() {
  Serial.begin(115200);
  Serial6.begin(115200);
  Wire.begin();
  Wire.setModule(0);
  ledsConfig();
  gyroConfig();
  motorConfig();
  encodersConfig();
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(SW1,INPUT);
  pinMode(SW2,INPUT);
}

void loop() {
  while (Serial6.available() > 0) {
    
    selector =  Serial6.read();
  }
  enviarSerial(selectorDato(selector - 48),6);
  enviarSerial(selectorDato(selector - 48),1);
  leergyro();
  
  delay(50);
  if(digitalRead(encA)){
      digitalWrite(LED1,HIGH);

  }
  else{
      digitalWrite(LED1,LOW);
  }
}

void ledsConfig(){
  analogReadResolution(12);
  pinMode(emisor1, OUTPUT); 
  pinMode(emisor3, OUTPUT);
  pinMode(emisor4, OUTPUT);
  pinMode(emisor6, OUTPUT);
}


void gyroConfig(){
  while(!gyro.init());
  gyro.enableDefault();
  
  for(int n=0;n<sampleNum;n++){
    gyro.read();
    dc_offset+=(int)gyro.g.x;
  }
  dc_offset=dc_offset/sampleNum;
  
  for(int n=0;n<sampleNum;n++){
  gyro.read();
  if((int)gyro.g.x-dc_offset>noise)
     noise=(int)gyro.g.x-dc_offset;
  else if((int)gyro.g.x-dc_offset<-noise)
     noise=-(int)gyro.g.x-dc_offset;
  }
  noise=noise/100; 
}



void encodersConfig() {
  pinMode(encA, INPUT);
  attachInterrupt(encA, encSumaA, CHANGE);    

}

void encSumaA() {
   ticksEncA++;
  
}



int medir(int n){
  int medicion=0;
  int E, R;
  int offsetOn = 0;  
  int offsetOff = 0; 
  switch(n){
    case 1: E=emisor1; R=receptor1; break; 
    case 3: E=emisor3; R=receptor3; break;
    case 4: E=emisor4; R=receptor4; break;
    case 6: E=emisor6; R=receptor6; break; 
  }
  digitalWrite(E,HIGH);
  delayMicroseconds(40);
  offsetOn = analogRead(R);  
  digitalWrite(E,LOW);    
  delayMicroseconds(100);
  offsetOff = analogRead(R); 
  medicion = offsetOn - offsetOff;
  return(medicion);
}

int selectorDato(int selector){
  int dato=0;
  switch(selector){
    case 1: dato=medir(1); break; 
    case 3: dato=medir(3); break;
    case 4: dato=medir(4); break;
    case 6: dato=medir(6); break;
    case 7: dato=(int)angle;     break;
    case 8: dato=(int)ticksEncA; break;
    case 2: dato=ticksEncB; break;
    default: dato=13; break;
    
    
  }
  return(dato);
}

void leergyro(){
  sampleTime = millis() - time;
  time = millis(); 
  gyro.read();
  rate=((int)gyro.g.x-dc_offset)/100;

  if(rate >= noise || rate <= -noise) angle += ((double)(prev_rate + rate) * sampleTime) / 2000;
  prev_rate = rate;
  
  if (angle < 0)
    angle += 360;
  else if (angle >= 360)
    angle -= 360;
}



void PWM_D(int velocidadD, int sentidoD) {
  if(velocidadD != 0){
    if (sentidoD == 0) {
      GEN_PWM(PWM_GEN_1,PWM_OUT_2,PWM_OUT_2_BIT,velocidadD);
      GEN_PWM(PWM_GEN_1,PWM_OUT_3,PWM_OUT_3_BIT,0);
    }
    else {
      GEN_PWM(PWM_GEN_1,PWM_OUT_2,PWM_OUT_2_BIT,0);
      GEN_PWM(PWM_GEN_1,PWM_OUT_3,PWM_OUT_3_BIT,velocidadD);
    }
  }
  else {
    GEN_PWM(PWM_GEN_1,PWM_OUT_2,PWM_OUT_2_BIT,0);
    GEN_PWM(PWM_GEN_1,PWM_OUT_3,PWM_OUT_3_BIT,0);
  }
}

void PWM_I(int velocidadI, int sentidoI) {
  if(velocidadI != 0){
    if (sentidoI == 0) {
      GEN_PWM(PWM_GEN_3,PWM_OUT_6,PWM_OUT_6_BIT,velocidadI);
      GEN_PWM(PWM_GEN_3,PWM_OUT_7,PWM_OUT_7_BIT,0);
    }
    else {
      GEN_PWM(PWM_GEN_3,PWM_OUT_6,PWM_OUT_6_BIT,0);
      GEN_PWM(PWM_GEN_3,PWM_OUT_7,PWM_OUT_7_BIT,velocidadI);
    }
  }
  else {
    GEN_PWM(PWM_GEN_3,PWM_OUT_6,PWM_OUT_6_BIT,0);
    GEN_PWM(PWM_GEN_3,PWM_OUT_7,PWM_OUT_7_BIT,0);
  }
}


void motorConfig() {
  
  
  pinMode(NSLEEP, OUTPUT);
  digitalWrite(NSLEEP, HIGH);

  SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1)) {}

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  GPIOPinConfigure(GPIO_PF2_M1PWM6);
  GPIOPinConfigure(GPIO_PF3_M1PWM7);
  GPIOPinConfigure(GPIO_PA6_M1PWM2);
  GPIOPinConfigure(GPIO_PA7_M1PWM3);


  GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);
  GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_7);
  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);

  PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN |
                  PWM_GEN_MODE_NO_SYNC);
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 2500);

  PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |
                  PWM_GEN_MODE_NO_SYNC);
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 2500);
}



void GEN_PWM(int GEN, int OUT,int OUT_BITS, int vel){
  PWMGenEnable(PWM1_BASE, GEN);
  PWMPulseWidthSet(PWM1_BASE, OUT, vel);
  PWMOutputState(PWM1_BASE, OUT_BITS, true);
}


void enviarSerial(int dato, int serial){
  char buffer[5];
  for(int i=0;i<4;i++){
    buffer[i]=dato>>i*8;  
  }
  buffer[4]=0xFF;
  for(int i=0;i<5;i++){
    if(serial==1)
      Serial.write(buffer[i]);
    else 
      Serial6.write(buffer[i]);
  }
}



