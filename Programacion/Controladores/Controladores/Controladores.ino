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
#include <LSM303.h>
#include <Wire.h>
#include <L3G.h>
///////////////////////////// VARIABLES GENERALES //////////////////////////////////////
#define LED PB_0
#define Dist_casilla 168
#define Dist_casilla_A 186
// Constante mm/ticks = 8.22 mm/ticks - 8.10mm/ticks - 8.57mm/ticks - 8.75mmm/ticks => 8.41mm/ticks
float mmxticks = 8.41;
int estado = 1;
int pwm = 20;
char buffer[9];
int serial;
int prueba_leds;
int RESET = 0;
int i;
////////////////////////// VARIABLES SENSORES INFRAROJOS ////////////////////////////////
//emisores
int emisor1=PB_4;
int emisor3=PD_6;
int emisor4=PD_0;
int emisor6=PE_4;

//receptores
int receptor1=PD_1;
int receptor3=PD_3;
int receptor4=PE_2;
int receptor6=PE_0;


int medidaD;
int medidaI;
int medidaDelanteraI;
int medidaDelanteraD;

////////////////////////////// VARIABLES ENCODERS ///////////////////////////////////////
#define encA PC_6                            //Encoder izquierdo fase A
#define encB PC_5                            //Encoder izquierdo fase B
volatile long ticksEncA = 0;                //Ticks de la fase A del encoder izquierdo
volatile long ticksEncB = 0;                //Ticks de la fase B del encoder izquierdo
/////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////Variables de paredes ////////////////////////////
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
int wallDetectDelanteraD = 1500;
int wallDetectDelanteraI = 1400;

/////////////////////////// Variables GYRO /////////////////////////////////
L3G gyro;
int sampleNum = 1000;
int dc_offset = 0;
double noise = 0;
unsigned long time;
int sampleTime = 10;
int rate;
int prev_rate = 0;
double angulo = 0;
int temp;
int anguloDeseado = 0;

/////////////////////////// Variables Acelerometro ////////////////////////////////
LSM303 compass;
//LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

int compass_offset = 0;
float aceleracionZ = 0;
float aceleracionZprevia = 0;
float velocidadZ = 0;
float velocidadZprevia = 0;
float distanciaZ = 0;
float distanciaZprevia = 0;
float ruidoAceleracion = 0;
float distanciaRecorrida = 0;
int contador = 0;

////////////////////////////// VARIABLES MOTORES ///////////////////////////////////////
#define GPIO_PF2_M1PWM6         0x00050805
#define GPIO_PF3_M1PWM7         0x00050C05
#define GPIO_PA6_M1PWM2         0x00001805
#define GPIO_PA7_M1PWM3         0x00001C05
#define NSLEEP PC_7
#define FAUTL PD_7
#define FORWARD 1
#define BACKWARD 0

#define pwmI_baseM 1300//880//1080
#define pwmD_baseM 1000//580//780

//////////////////////////////// VARIABLES CONTROLADORES ////////////////////////////////
double KpForward = 0.8; /*constante propocional 0.9*/
double KdForward = 1; /*constante integradora 0.4*/
float errorPForward, errorDForward, oldErrorForward;
float errorTotalForward;

float KpCruce = 0.8;
float KdCruce = 1;
float errorDcruce = 0;
float errorPcruce = 0;
float errorTotalCruce = 0;
float oldErrorPcruce = 0;
/////////////////////////////////////////////////////////////////////////////////////////

///////////////////////// Otras Variables//////////////////////////////
int LED1 = PA_2;
int LED2 = PA_3;
int inicio=0;
int selector = 89;

void ledsConfig();
void gyroConfig();
void motorConfig();
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
void enviarWifi();
void caminarMientrasHayPared();
void calcularAcelerometro();
void acelConfig();
void girarAngulo(int sentido, int maxTimeMillis);
void alinearFrenar(int maxTimeMillis, int distanciaDeseadaI, int distanciaDeseadaD);


void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial6.begin(115200);
  Wire.begin();
  Wire.setModule(0);
  ledsConfig();
  gyroConfig();
  motorConfig();
  //acelConfig();
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);

  calOffsetD = medir(3);
  calOffsetI = medir(4);
  calOffsetID = calOffsetD - calOffsetI;

  digitalWrite(LED1,HIGH);
  delay(200);
  digitalWrite(LED1,LOW);
  delay(200);
}

void loop() {
  //digitalWrite(LED2,HIGH);
  //PWM_D(pwmD_baseM,FORWARD);
  //delay(500);
  //digitalWrite(LED2,LOW);
  //PWM_D(0,FORWARD);
  //delay(500);

  
  //NO BORRAR, solo comentar (avamza y se detiene cuando encuentra pared
  /*medidaDelanteraI = medir(6);
  medidaDelanteraD = medir(1);

  if(medidaDelanteraI > 400 && medidaDelanteraD > 400 ){
      alinearFrenar(1000,detenerParedI,detenerParedD);
      //CRUZAR
  }
  else{
    fowardNPasos(0,100);
  }
      //while(1){};
      /*newFrenado(200);
      digitalWrite(LED1,HIGH);
      delay(2000);
      digitalWrite(LED1,LOW);
      medidaD = medir(3);
      medidaI = medir(4);
      
      
      if (medidaD > wallDetectD){//hay pared derecha
        girarAngulo(0,1000);
      }
      else if (medidaI > wallDetectI){//hay pared izquierda
        girarAngulo(1,1000);
      }
      else{
        while(1);
      }
   }*/
   
  
  girarAngulo(0,1000);
  delay(3000);
  //enviarWifi();
  //leergyro();
    
}


//********************************** Setup IR *******************
void ledsConfig(){
  analogReadResolution(12);// ajuste ADC en 12 bits
  pinMode(emisor1, OUTPUT);
  pinMode(emisor3, OUTPUT);
  pinMode(emisor4, OUTPUT);
  pinMode(emisor6, OUTPUT);
  calOffsetD = medir(3);
  calOffsetI = medir(4);
  calOffsetID = calOffsetD - calOffsetI;
}

//************************ Setup Gyro ***********************
void gyroConfig(){
  while(!gyro.init());
  gyro.enableDefault();
  //Calculate initial DC offset and noise level of gyro
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
  noise=noise/100; //gyro returns hundredths of degrees/sec
}


int medir(int n){
  int medicion=0;
  int E, R;
  int offsetOn = 0;  // medida con los led prendidos
  int offsetOff = 0; // medida con los led apagados
  switch(n){
    case 1: E=emisor1; R=receptor1; break;
    case 3: E=emisor3; R=receptor3; break;
    case 4: E=emisor4; R=receptor4; break;
    case 6: E=emisor6; R=receptor6; break;
  }
  digitalWrite(E,HIGH);// encendido emisor
  delayMicroseconds(40);
  offsetOn = analogRead(R);  // lectura ADC
  digitalWrite(E,LOW);    // apagado emisor
  delayMicroseconds(100);
  offsetOff = analogRead(R); // lectura ADC
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
    case 9: dato=(int)contador; break;
    default: dato=14; break;
    //case 7: dato=gyro;                 //Gyro
    //case 8: dato=paredes;              //Actualizar paredes
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
  // Keep our angle between 0-359 degrees
  if (angulo < 0)
    angulo += 360;
  else if (angulo >= 360)
    angulo -= 360;
}

// Las primeras dos funciones controlan un motor, el primer paramatro recibido es la
//velocidad y el segundo es el sentido en el que se desea mover: 1 DELANTE, 0 ATRAS
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

//****************************** SETUP MOTORES ****************************************
void motorConfig() {
  //pinMode(BIN2, OUTPUT);
  //pinMode(AIN2, OUTPUT);
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

///////////////////////////CONTROLADORES////////////////////////////////

void fowardNPasos(int cantidadPasos, int baseVel){
  //while(Ticks_Enc_Der_A <= ((cantidadPasos * Dist_casilla)/mmxticks)){
    medidaD = medir(3);
    medidaI = medir(4);
    forwardPID(baseVel);
  //}
}

void forwardPID(int base){
    //CASO 0: HAY DOS PAREDES
    if (medidaD > wallDetectD && medidaI > wallDetectI ){
       errorPForward = medidaD - medidaI - calOffsetID;
       errorDForward = errorPForward - oldErrorForward;
    }
    //CASO 1: NO HAY PARED DERECHA
    else if ( medidaD < wallDetectD ){
       errorPForward = (calOffsetI - medidaI);
       errorDForward = errorPForward - oldErrorForward;
    }
    //CASO 2: NO HAY PARED IZQUIERDA
    else if ( medidaI < wallDetectI ){
       errorPForward = (medidaD - calOffsetD);
       errorDForward = errorPForward - oldErrorForward;
    }
    // CASO 3: NO HAY PAREDES

    errorTotalForward = KpForward * errorPForward + KdForward * errorDForward;
    oldErrorForward = errorPForward;
    PWM_I(pwmI_baseM - base - (int)errorTotalForward,FORWARD);
    PWM_D(pwmD_baseM - base + (int)errorTotalForward,FORWARD);
}

void cruceSuave(int sentido){
  //Sentido -> 0 horario
  //Sentido -> 1 antihorario
  //Establecer el angulo de gyro
  int cruce = 0;
  leergyro();
  if(sentido==0)
    anguloDeseado = angulo - 90;
  else
    anguloDeseado = angulo + 90;
  // Mantener el angulo entre 0-359 grados
  if (anguloDeseado < 0)
    anguloDeseado += 360;
  else if (anguloDeseado >= 360)
     anguloDeseado -= 360;

  while(!cruce){
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

    if(errorPcruce == 0){
      cruce = 1;
    }
  }
  digitalWrite(LED2,HIGH);
  delay(500);
  digitalWrite(LED2,LOW);

}

void enviarWifi(){
  while (Serial6.available() > 0) {
    // read the incoming byte:
    selector =  Serial6.read();
  }
  enviarSerial(selectorDato(selector - 48),6);
  enviarSerial(selectorDato(selector - 48),1);
}


void caminarMientrasHayPared(){
  enviarWifi();
  calcularAcelerometro();
  medidaD = medir(3);
  medidaI = medir(4);
  if(distanciaZ <= 46000 && contador < 3){
    fowardNPasos(0,0);    
  }
  /*if (medidaD > wallDetectD || medidaI > wallDetectI ){
    fowardNPasos(0,0);
  }*/
  else{
    //frenar();
    aceleracionZ = 0;
    aceleracionZprevia = 0;
    velocidadZ = 0;
    velocidadZprevia = 0;
    distanciaZ = 0;
    distanciaZprevia = 0;
    ruidoAceleracion = 0;
    distanciaRecorrida = 0;
    contador += 1;
  }
}

void calcularAcelerometro(){
  //Calibracion
  compass.read();
  int deltat = millis() - time;
  time = millis();
  float temp;
  aceleracionZ = ((int)(compass.a.z >> 4)-compass_offset) *9.8/1024;
  aceleracionZ = 0.6*aceleracionZ+0.4*aceleracionZprevia;
  if(aceleracionZ >= ruidoAceleracion || aceleracionZ <= -ruidoAceleracion) {
    velocidadZ += aceleracionZ*deltat;
    velocidadZ = 0.6*velocidadZ+0.4*velocidadZprevia;
    distanciaZ += velocidadZ*deltat;
    distanciaZ = 0.6*distanciaZ+0.4*distanciaZprevia;
    }//angle += ((double)(prev_rate + rate) * sampleTime) / 2000;
    aceleracionZprevia = aceleracionZ;
    velocidadZprevia = velocidadZ;
    distanciaZprevia = distanciaZ;
  //prev_rate = rate;
  // Keep our angle between 0-359 degrees
}

void acelConfig(){
  compass.init();
  compass.enableDefault();

  for(int n=0;n<sampleNum;n++){
    compass.read();
    compass_offset+=(int)(compass.a.z >> 4);
  }
  compass_offset=compass_offset/sampleNum;

  for(int n=0;n<sampleNum;n++){
  compass.read();
  if((int)(compass.a.z >> 4)-compass_offset>ruidoAceleracion)
     ruidoAceleracion = (int)(compass.a.z >> 4)-compass_offset;
  else if((int)gyro.g.x-dc_offset<-ruidoAceleracion)
     ruidoAceleracion = -(int)(compass.a.z >> 4)-compass_offset;
  }
  ruidoAceleracion = ruidoAceleracion*9.8/1024;
}

////////////////////Controlador de alineacion (distancia) frontal/////////////////////
int detenerParedI = 2200;
int detenerParedD = 2200;
int distancia;
int distanciaAnterior;
float detenerPared = ( detenerParedI + detenerParedD ) /2;
double kpDetenerse = 100;
double kdDetenerse = 0;
double kpVel = 1;
double kdVel = 0;
double kpDis = 10;
double kdDis = 0;
float errorHayparedI, errorHayparedD, oldErrorHayparedI, oldErrorHayparedD;
float errorVel;
float errorDistancia;
int velFrenadoI, velFrenadoD;
int distanciaParedI, distanciaParedD;



void alinearFrenar(int maxTimeMillis, int distanciaDeseadaI, int distanciaDeseadaD){
  int timeCount = 0;
  distanciaAnterior = 0;
  distancia = 0;
  velFrenadoI = pwmI_baseM;
  velFrenadoD = pwmD_baseM;
  detenerParedI = distanciaDeseadaI;
  detenerParedD = distanciaDeseadaD;
  while(1){
    distanciaParedI = medir(6);
    distanciaParedD = medir(1);
    distancia = (distanciaParedI + distanciaParedD)/2;
    errorDistancia = distancia - distanciaAnterior;
    //Error de diferencia de velocidad
    errorVel = distanciaParedI-distanciaParedD;    
    //Cuanto falta para la pared
    errorHayparedI = (detenerParedI - distanciaParedI);
    velFrenadoI += errorHayparedI*kpDetenerse + (errorHayparedI - oldErrorHayparedI)*kdDetenerse - errorVel*kpVel - errorVel*kdVel;
    if(velFrenadoI > 0){            // Si estoy antes de la referencia avanzo reduciendo la velocidad
      PWM_I(velFrenadoI,FORWARD);
    }
    else{                           // Si me pase de la referencia retrocedo
      PWM_I(abs(velFrenadoI)+errorDistancia*kpDis,BACKWARD);
    }  
    oldErrorHayparedI = errorHayparedI;    

    
    errorHayparedD = detenerParedD - distanciaParedD;
    velFrenadoD += errorHayparedD*kpDetenerse + (errorHayparedD - oldErrorHayparedD)*kdDetenerse + errorVel*kpVel + errorVel*kdVel;
    if(velFrenadoD > 0){                  // Si estoy antes de la referencia avanzo reduciendo la velocidad
      PWM_D(velFrenadoD,FORWARD);
    }
    else{
      PWM_D(abs(velFrenadoD)+errorDistancia*kpDis,BACKWARD);   // Si me pase de la referencia retrocedo
    }
    oldErrorHayparedD = errorHayparedD;

    distanciaAnterior = distancia;
    delay(10);
    timeCount += 10;
    if(errorHayparedD < 40 && errorHayparedI < 40 && timeCount > maxTimeMillis){
      break;
    }
  }
}

void girarAngulo(int sentido, int maxTimeMillis){
    leergyro();

    int timeCount = 0;
    int timeZero = millis();
     if (sentido == 0){
        PWM_I(pwmI_baseM + 100,0);
        PWM_D(pwmD_baseM ,1);
      }
      else{
        PWM_I(pwmI_baseM ,1);
        PWM_D(pwmD_baseM + 100,0);
      }
      delay(300);
    while(angulo <= 90 || angulo >= 270  || timeCount < maxTimeMillis){
      
      if (sentido == 0){
        errorPcruce = 90 - angulo;
        errorDcruce = errorPcruce - oldErrorPcruce;

        errorTotalCruce = KpCruce * errorPcruce + KdCruce * errorDcruce;

        if (errorTotalCruce > 0){
          PWM_I(pwmI_baseM + (int)errorTotalCruce +100,0);
          PWM_D(pwmD_baseM + (int)errorTotalCruce,1);
        }
        else{
          PWM_I(pwmI_baseM  + (int)errorTotalCruce ,1);
          PWM_D(pwmD_baseM + (int)errorTotalCruce + 100,0);
        }
      }
      else{
        angulo = angulo == 0 ? 360 : angulo;
        errorPcruce = angulo - 270;
        errorDcruce = errorPcruce - oldErrorPcruce;

        errorTotalCruce = KpCruce * errorPcruce + KdCruce * errorDcruce;
        
        if (errorTotalCruce > 0){
          PWM_I(pwmI_baseM  + (int)errorTotalCruce,1);
          PWM_D(pwmD_baseM +(int)errorTotalCruce + 100,0);
        }
        else{
          PWM_I(pwmI_baseM + (int)errorTotalCruce + 100,0);
          PWM_D(pwmD_baseM + (int)errorTotalCruce,1);
        }
      }
      digitalWrite(LED1,HIGH);
      leergyro();
      delay(10);
      timeCount = millis() - timeZero; 
    }
    digitalWrite(LED1,LOW);
    angulo = 0;
    rate = 0;
    prev_rate = 0;
    time = 0;
    /*if (sentido == 0){
      PWM_I(pwmI_baseM+500 ,1);
      PWM_D(pwmD_baseM+500 ,0);
    }
    else{
      PWM_I(pwmI_baseM+500, 0);
      PWM_D(pwmD_baseM+500, 1);
    }
    delay(20);*/
    PWM_I(0  /*+ (int)errorTotalCruce*/,1);
    PWM_D(0 /*+ (int)errorTotalCruce*/,0);
}
