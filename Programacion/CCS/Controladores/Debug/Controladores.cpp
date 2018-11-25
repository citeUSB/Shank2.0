#include "Energia.h"

#line 1 "C:/Users/Andres/Documents/GitHub/Shank2.0/Programacion/CCS/Controladores/Controladores.ino"
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
int medir(int n);
int selectorDato(int selector);
void leergyro();
void PWM_D(int velocidadD, int sentidoD);
void PWM_I(int velocidadI, int sentidoI);
void motorConfig();
void GEN_PWM(int GEN, int OUT,int OUT_BITS, int vel);
void enviarSerial(int dato, int serial);
void fowardNPasos(int cantidadPasos, int baseVel);
void forwardPID(int base);
void cruceSuave(int sentido);
void frenar();
void enviarWifi();

#line 18
float mmxticks = 8.41;
int estado = 1;
int pwm = 20;
char buffer[9];
int serial;  
int prueba_leds;
int RESET = 0;    
int i;


int emisor1=PB_4;
int emisor3=PD_6;
int emisor4=PD_0;
int emisor6=PE_4;


int receptor1=PD_1;
int receptor3=PD_3;
int receptor4=PE_2;
int receptor6=PE_0;


int medidaD;
int medidaI;
int medidaDelanteraI;
int medidaDelanteraD;


#define encA PC_6                            
#define encB PC_5                            
volatile long ticksEncA = 0;                
volatile long ticksEncB = 0;                



int Pared_D;
int Pared_I;
int Pared_De;
int Pared_DiD;
int Pared_DiI;

int wallDetectD = 1700;
int wallDetectI = 1300;
int calOffsetD = 2690;
int calOffsetI = 2200;
int calOffsetID = 590;
int wallDetectDelanteraD = 950;
int wallDetectDelanteraI = 600;


L3G gyro;
int sampleNum = 5;
int dc_offset = 0;
double noise = 0;
unsigned long time;
int sampleTime = 10;
int rate;
int prev_rate = 0;
double angulo = 0;
int temp;
int anguloDeseado = 0;

#define GPIO_PF2_M1PWM6         0x00050805
#define GPIO_PF3_M1PWM7         0x00050C05
#define GPIO_PA6_M1PWM2         0x00001805
#define GPIO_PA7_M1PWM3         0x00001C05
#define NSLEEP PC_7
#define FAUTL PD_7
#define FORWARD 1
#define BACKWARD 0

#define pwmI_baseM 1080
#define pwmD_baseM 780


double KpForward = 0.8;  
double KdForward = 1; 
float errorPForward, errorDForward, oldErrorForward;
float errorTotalForward;

float KpCruce = 1;
float KdCruce = 0;
float errorDcruce = 0;
float errorPcruce = 0;
float errorTotalCruce = 0;
float oldErrorPcruce = 0;



int LED1 = PA_2;
int LED2 = PA_3;
int inicio=0;
int selector = 89;


void setup() {
  Serial.begin(115200);
  Serial6.begin(115200);
  Wire.begin();
  Wire.setModule(0);
  ledsConfig();
  gyroConfig();
  motorConfig();
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  
  digitalWrite(LED1,HIGH);
  delay(1000);
  digitalWrite(LED1,LOW);
  delay(1000);  
}

void loop() {
  enviarWifi();
  
 








 
  cruceSuave(0);
  digitalWrite(LED2,HIGH);
  delay(1000);
  digitalWrite(LED2,LOW);
  cruceSuave(1);
  digitalWrite(LED2,HIGH);
  delay(1000);
  digitalWrite(LED2,LOW);
  
  
  

}

void ledsConfig(){
  analogReadResolution(12);
  pinMode(emisor1, OUTPUT); 
  pinMode(emisor3, OUTPUT);
  pinMode(emisor4, OUTPUT);
  pinMode(emisor6, OUTPUT);
  calOffsetD = medir(3);
  calOffsetI = medir(4);
  calOffsetID = calOffsetD - calOffsetI;
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
    case 7: dato=(int)angulo;     break;
    case 8: dato=(int)errorPcruce; break;
    case 9: dato=(int)errorPcruce; break;
    default: dato=13; break;
    
    
  }
  return(dato);
}

void leergyro(){
  sampleTime = millis() - time;
  time = millis(); 
  gyro.read();
  rate=((int)gyro.g.x-dc_offset)/100;

  if(rate >= noise || rate <= -noise) angulo += ((double)(prev_rate + rate) * sampleTime) / 2000;
  prev_rate = rate;
  
  if (angulo < 0)
    angulo += 360;
  else if (angulo >= 360)
    angulo -= 360;
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



void fowardNPasos(int cantidadPasos, int baseVel){
  
    medidaD = medir(3);
    medidaI = medir(4); 
    forwardPID(baseVel);
  
}

void forwardPID(int base){  
    
    if (medidaD > wallDetectD && medidaI > wallDetectI ){
       errorPForward = medidaD - medidaI - calOffsetID;
       errorDForward = errorPForward - oldErrorForward;
    }
    
    else if ( medidaD < wallDetectD ){
       errorPForward = (calOffsetI - medidaI);
       errorDForward = errorPForward - oldErrorForward;
    }
    
    else if ( medidaI < wallDetectI ){
       errorPForward = (medidaD - calOffsetD);
       errorDForward = errorPForward - oldErrorForward;
    }
    
    
    errorTotalForward = KpForward * errorPForward + KdForward * errorDForward;
    oldErrorForward = errorPForward;
    PWM_I(pwmI_baseM - base - (int)errorTotalForward,FORWARD);
    PWM_D(pwmD_baseM - base + (int)errorTotalForward,FORWARD);
}

void cruceSuave(int sentido){
  
  
  
  int cruce = 0;
  leergyro();
  if(sentido==0)
    anguloDeseado = angulo - 90; 
  else  
    anguloDeseado = angulo + 90; 
  
  if (anguloDeseado < 0)
    anguloDeseado += 360;
  else if (anguloDeseado >= 360)
     anguloDeseado -= 360;
 
  while(cruce){
    leergyro();
    errorPcruce = anguloDeseado - angulo;
    enviarWifi();
    if(errorPcruce > 180)
    errorPcruce -= 360;
    else if(errorPcruce < -180)
    errorPcruce += 360;
    
    errorDcruce = errorPcruce - oldErrorPcruce;
    
    errorTotalCruce = KpCruce * errorPcruce + KdCruce * errorDcruce;
    oldErrorPcruce = errorPcruce;   
    PWM_I(pwmI_baseM - 100 - (int)errorTotalCruce,!sentido);
    PWM_D(pwmD_baseM - 100 + (int)errorTotalCruce,sentido);

    if(errorPcruce < 5)
      cruce = 1;

  }
  
}

void frenar(){
  PWM_I(pwmI_baseM + 300,BACKWARD);
  PWM_D(pwmD_baseM + 300,BACKWARD);
  delay(200);
  PWM_I(0,FORWARD);
  PWM_D(0,FORWARD);
}

void enviarWifi(){
  while (Serial6.available() > 0) {
    
    selector =  Serial6.read();
  }
  enviarSerial(selectorDato(selector - 48),6);
  enviarSerial(selectorDato(selector - 48),1);
}



