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
int ticksPorCasilla = 85;
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
#define enc PF_0                            //Encoder izquierdo fase A
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

/////////////////////////////////////////////////////////////////////////////////////////

///////////////////////// Otras Variables//////////////////////////////
int LED1 = PA_2;
int LED2 = PA_3;
//int inicio=0;
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


#include "coord.h"
#include "entry.h"
#include "StackList.h"
#include "QueueList.h"
#include "instruction.h"

//Coordenadas Ejes Globales

#define X 6
#define Y 6

coord inicio = {0,0};

//Cardinales
byte cardinalGlobal = 4;

//Cardinales N,S,E,W
byte cardinales[] = { 1,2,4,8 };


//Instrucciones de navegacion
QueueList <instruction> navegacion;

/*Enumerstor de las instrucciones de navegacion
8 = avanzar casilla
1 = girar sentido horario 90°
0 = girar en sentido antihorario 90°
*/
// Constantes de LEDS
//int Media_L = 1, DiagonalD = 1, DiagonalI = 1, DelanteroD = 1, DelanteroI = 1, ParedD = 1, ParedI = 1;             ////////////////////////EDITAR

// Test de generacion del maze (distancia y walls)

// Descripcion walls

/*
NSEW = 0
-SEW = 1
N-EW = 2
NS-W = 4
NSE- = 8
NS-- = 12
--EW = 3
N--W = 6
N-E- = 10
-S-W = 5
-SE- = 9
N--- = 14
-S-- = 13
--E- = 11
---W = 7
---- = 15
*/

//mapa
entry maze[X][Y];

//Coordenadas GLobales
coord globalCoord = { 0,0 };
coord globalEnd = { 0,0 };

// the setup function runs once when you press reset or power the board

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
  encodersConfig();
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);

  calOffsetD = medir(3);
  calOffsetI = medir(4);
  calOffsetID = calOffsetD - calOffsetI;

  iniciar_maze();

  digitalWrite(LED1,HIGH);
  delay(200);
  digitalWrite(LED1,LOW);
  delay(200);
  
}

//****** Setup Encoders ********
int encCount = 0;

void encodersConfig() {
  pinMode(enc, INPUT);
  attachInterrupt(enc, encSuma, CHANGE);    // Interrupcion del canal del encoder

}

void encSuma(){
  encCount++;
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
    case 8: dato=(int)encCount; break;
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
  while(encCount <= ticksPorCasilla*cantidadPasos){
    medidaD = medir(3);
    medidaI = medir(4);
    forwardPID(baseVel);
  }
  encCount = 0;
  PWM_I(pwmI_baseM + 500,BACKWARD);
  PWM_D(pwmD_baseM + 500,BACKWARD);
  delay(100);
  PWM_I(0,BACKWARD);
  PWM_D(0,BACKWARD);
}

void forwardPID(int base){
    //CASO 0: HAY DOS PAREDES
    if (medidaD > wallDetectD -100 && medidaI > wallDetectI-100 ){
       errorPForward = medidaD - medidaI - calOffsetID;
       errorDForward = errorPForward - oldErrorForward;

       errorTotalForward = KpForward * errorPForward + KdForward * errorDForward;
       oldErrorForward = errorPForward;
       PWM_I(pwmI_baseM - base - (int)errorTotalForward,FORWARD);
       PWM_D(pwmD_baseM - base + (int)errorTotalForward,FORWARD);
    }
    //CASO 1: NO HAY PARED DERECHA
    else if ( medidaD < wallDetectD-100 && medidaI > wallDetectI-100){
       errorPForward = (calOffsetI - medidaI);
       errorDForward = errorPForward - oldErrorForward;

       errorTotalForward = KpForward * errorPForward + KdForward * errorDForward;
       oldErrorForward = errorPForward;
       PWM_I(pwmI_baseM - base - (int)errorTotalForward,FORWARD);
       PWM_D(pwmD_baseM - base + (int)errorTotalForward,FORWARD);
    }
    //CASO 2: NO HAY PARED IZQUIERDA
    else if ( medidaI < wallDetectI-100 && medidaD > wallDetectD -100){
       errorPForward = (medidaD - calOffsetD);
       errorDForward = errorPForward - oldErrorForward;
       
       errorTotalForward = KpForward * errorPForward + KdForward * errorDForward;
       oldErrorForward = errorPForward;
       PWM_I(pwmI_baseM - base - (int)errorTotalForward,FORWARD);
       PWM_D(pwmD_baseM - base + (int)errorTotalForward,FORWARD);
    }
    // CASO 3: NO HAY PAREDES
    else{
      PWM_I(1300,FORWARD);
      PWM_D(1300,FORWARD);
    }

    
}

void enviarWifi(){
  while (Serial6.available() > 0) {
    // read the incoming byte:
    selector =  Serial6.read();
  }
  enviarSerial(selectorDato(selector - 48),6);
  enviarSerial(selectorDato(selector - 48),1);
}
//////////////////////////////
float KpCruce = 24;
float KdCruce = 0.1;
float errorDcruce = 0;
float errorPcruce = 0;
float errorTotalCruce = 0;
float oldErrorPcruce = 0;

void girarAngulo(int sentido, int maxTimeMillis){
    leergyro();
    int tempi = 0;
    int inicial = 500;//350 madera
    int timeCount = 0;
    int timeZero = millis();
     if (sentido == 0){
        PWM_I(pwmI_baseM + inicial,0);
        PWM_D(pwmD_baseM + inicial ,1);
      }
      else{
        PWM_I(pwmI_baseM + inicial,1);
        PWM_D(pwmD_baseM + inicial,0);
      }
      delay(50);
      while(angulo <= 60 || angulo >= 295 || timeCount < maxTimeMillis){
        if (sentido == 0){
          errorPcruce = 90 - angulo;
          errorDcruce = errorPcruce - oldErrorPcruce;
  
          errorTotalCruce = (KpCruce * errorPcruce) + (KdCruce * errorDcruce);
  
          if (errorTotalCruce > 0){
            PWM_I((int)abs(errorTotalCruce) - tempi,0);
            PWM_D((int)abs(errorTotalCruce) - tempi,1);
          }
          else{
            PWM_I((int)abs(errorTotalCruce) - tempi ,1);
            PWM_D((int)abs(errorTotalCruce) - tempi,0);
          }
        }
        else{
          angulo = angulo == 0 ? 360 : angulo;
          errorPcruce = angulo - 270;
          errorDcruce = errorPcruce - oldErrorPcruce;
  
          errorTotalCruce = KpCruce * errorPcruce + KdCruce * errorDcruce;
  
          if (errorTotalCruce > 0){
            PWM_I((int)abs(errorTotalCruce) - tempi,1);
            PWM_D((int)abs(errorTotalCruce) - tempi,0);
          }
          else{
            PWM_I((int)abs(errorTotalCruce) - tempi,0);
            PWM_D((int)abs(errorTotalCruce) - tempi,1);
          }
      }
      leergyro();
      delay(10);
      timeCount = millis() - timeZero;

      if (sentido == 0){
        digitalWrite(LED1,HIGH);
      }
      else{
        digitalWrite(LED2,HIGH);
      }
    }
    digitalWrite(LED1,LOW);
    digitalWrite(LED2,LOW);
    angulo = 0;
    rate = 0;
    prev_rate = 0;
    time = 0;
    if (sentido == 0){
      PWM_I(pwmI_baseM+500 ,FORWARD);
      PWM_D(pwmD_baseM+500 ,BACKWARD);
    }
    else{
      PWM_I(pwmI_baseM+500, BACKWARD);
      PWM_D(pwmD_baseM+500, FORWARD);
    }
    delay(20);
    PWM_I(0  /*+ (int)errorTotalCruce*/,1);
    PWM_D(0 /*+ (int)errorTotalCruce*/,0);
    encCount = 0;
}


//###### CODIGO FLOOD FILL

void printMazewalls(){
  for(int j=0; j<Y; j++){
    for(int i=0; i<X; i++){
      Serial.print(maze[j][i].walls);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void printMazedistancia(){
  for(int j=0; j<Y; j++){
    for(int i=0; i<X; i++){
      Serial.print(maze[j][i].distancia);
      Serial.print(" ");
    }
    Serial.println();
  }
}

//Calcula la distancia mas optima entre dos puntos en el arreglo sin paredes
int calcDist(int posx, int posy, int destinoX, int destinoY) {
  int dist = (int)(abs(destinoY - posy) + abs(destinoX - posx));
  return dist;
}


//Chequear con las esquinas si la coordenada esta dentro o fuera del maze
//Si la coordena esta fuera del maze chequear_esquina retorna false

boolean chequear_esquina(coord actual) {
  if ((actual.x >= X) || (actual.y >= Y) || (actual.x < 0) || (actual.y < 0)) {
    return false;
  }
  else {
    return true;
  }
}

// Calcular distacia al centro 
int calcCentro(int posx, int posy, int dim) {
  int centro = dim / 2;
  int dist = 0;

  if (posy<centro) {
    if (posx<centro) {
      //Si estas en la esquina superior izquierda del maze
      dist = calcDist(posx, posy, (centro - 1), (centro - 1));
    }
    else {
      //Si estas en la esquina superior derecha del maze
      dist = calcDist(posx, posy, centro, (centro - 1));
    }
  }
  else {
    if (posx >= centro) {
      //Si estás en la parte inferior derecha del maze
      dist = calcDist(posx, posy, centro, centro);
    }
    else {
      //Si estás en la parte inferior izquierda del maze
      dist = calcDist(posx, posy, (centro - 1), centro);
    }
  }
  return dist;
}

void iniciar_maze() {
  int j;
  int i;
  maze[0][0].walls = 6;
  maze[Y - 1][0].walls = 5;
  maze[0][X - 1].walls = 10;
  maze[X - 1][Y - 1].walls = 9;
  for (j = 0; j<Y; j++) {
    for (i = 0; i<X; i++) {
      maze[j][i].check = 0;
      maze[j][i].distancia = calcCentro(i, j, X);
      if (j != 0 && j != Y - 1 && i != 0 && i != X - 1) {
        maze[j][i].walls = 15;
      }
      //Si es la columna izquierda (0,x)
      if (i == 0 && j != 0 && j != Y - 1) {
        maze[j][i].walls = 7;
      }
      //Si es la fila superior
      if (j == 0 && i != 0 && i != X - 1) {
        maze[j][i].walls = 14;
      }
      //si es la fila inferior
      if (j == (Y - 1) && i != X - 1 && i != 0) {
        maze[j][i].walls = 13;
      }
      //Si es la columna de la derecha
      if (i == (X - 1) && j != Y - 1 && j != 0) {
        maze[j][i].walls = 11;
      }
    }
  }
}

//INPUT: Coordenada actual ,  cardinal
//OUTPUT: Coordenada 
coord coordVecina(coord actual, byte cardinal) {
  coord sig_coord = { 0,0 };
  switch (cardinal) {
  case 1:
    //Me dirijo hacia el Norte
    sig_coord.x = actual.x;
    sig_coord.y = actual.y - 1;
    break;
  case 2:
    //Me dirijo hacia el Sur
    sig_coord.x = actual.x;
    sig_coord.y = actual.y + 1;
    break;
  case 4:
    //Me dirijo hacia el Este
    sig_coord.x = actual.x + 1;
    sig_coord.y = actual.y;
    break;
  case 8:
    //Me dirijo hacia el Oeste
    sig_coord.x = actual.x - 1;
    sig_coord.y = actual.y;
    break;
  }
  return sig_coord;
}

/*
INPUT: void
OUTOUT: Retorna un int que indica las paredes de la casilla actual 
*/
byte Haypared() {
  byte casilla = 15;
  byte norte = 0;
  byte sur = 0;
  byte este = 0;
  byte oeste = 0;
  switch (cardinalGlobal) {
  case 1:
    Serial.println("Cardinal Global hacia el norte");
    //Si hay pared al frente
    //Senseores frente
    Serial.println("Sensor frente");
    if ((medir(1) + medir(6))/2 > (wallDetectDelanteraD + wallDetectDelanteraD )/2) {
      //Norte
      norte = 1;
    }
    //Si hay pared a la derecha del carrito
    //Sensor derecho
    Serial.println("Sensor derecho");
    if (medir(3)>wallDetectD) {
      //Este
      este = 4;
    }
    //Si hay Pared a la izquierda del carrito
    //Sensor izquierdo
    Serial.println("Sensor izquierdo");
    if (medir(4)>wallDetectI) {
      //Oeste
      oeste = 8;
    }
    //15 - Valor obtenido
    casilla -= (norte + este + oeste);
    break;
  case 2:
    Serial.println("Cardinal Global hacia el sur");
    //Si hay pared al frente
    //Senseores frente
    Serial.println("Sensor frente");
    if ((medir(1) + medir(6))/2 > (wallDetectDelanteraD + wallDetectDelanteraD )/2) {
      //Sur
      sur = 2;
    }
    //Si hay pared a la derecha del carrito
    //Sensor derecho
    Serial.println("Sensor derecho");
    if (medir(3)>wallDetectD) {
      //Oeste
      oeste = 8;
    }
    //Si hay pared a la izquierda del carrito
    //Sensor izquierdo
    Serial.println("Sensor izquierdo");
    if (medir(4)>wallDetectI) {
      //Este     
      este = 4;
    }
    //15 - valor obtenido
    casilla -= (sur + este + oeste);
    break;
  case 4:
    Serial.println("Cardinal Global hacia el este");
    //Si hay pared al frente
    //Senseores frente
    Serial.println("Sensor frente");
    if ((medir(1) + medir(6))/2 > (wallDetectDelanteraD + wallDetectDelanteraD )/2) {
      //Este
      este = 4;
    }
    //Si hay pared a la derecha del carrito
    //Sensor derecho
    Serial.println("Sensor derecho");
    if (medir(3)>wallDetectD) {
      //Sur
      sur = 2;
    }
    //Si Hay pared a la izquierda del carrito
    //Sensor izquierdo
    Serial.println("Sensor izquierdo");
    if (medir(4)>wallDetectI) {
      //Norte
      norte = 1;
    }
    //15 - valor obtenido
    casilla -= (norte + sur + este);
    break;
  case 8:
    Serial.println("Cardinal Global hacia el oeste");
    //Si hay pared al frente
    //Senseores frente
    Serial.println("Sensor frente");
    if ((medir(1) + medir(6))/2 > (wallDetectDelanteraD + wallDetectDelanteraD )/2) {
      //Oeste
      oeste = 8;
    }
    //Si Hay pared a la derecha del carrito
    //Sensor derecho
    Serial.println("Sensor derecho");
    if (medir(3)>wallDetectD) {
      //Norte
      norte = 1;
    }
    //Si hay pared a la izquierda1 del carrito
    //Sensor izquierdo
    Serial.println("Sensor izquierdo");
    if (medir(4)>wallDetectI) {
      //Sur
      sur = 2;
    }
    //15 - Valor obtenido
    casilla -= (oeste + norte + sur);
    break;
  }
  return casilla;
}

/*
INPUT: Coordenada actual , cardinal
OUTPUT: Optima direccion en funcion de la coordenada.
*/

byte orientar(coord actual, byte cardinal) {
  coord sig_menor = { 0,0 };
  //El valor mas largo posible (dimension de la matriz)
  int sig_menor_val = X*Y;
  byte sig_dir = cardinal;
  //Si al frente esta disponible seguir
  if ((maze[actual.y][actual.x].walls & cardinal) != 0){
     coord sig_temp = coordVecina(actual, cardinal);    
     if(chequear_esquina(sig_temp)){
       sig_menor = sig_temp;
       sig_menor_val = maze[sig_menor.y][sig_menor.x].distancia;
     }
  }
  
  for (int i = 0; i<sizeof(cardinales); i++) {
    byte dir = cardinales[i];
    //Si la direccion es accesible
    if ((maze[actual.y][actual.x].walls & dir) != 0) {    
      //definimos la coord para la direccion
      coord dirCoord = coordVecina(actual, dir);
      //Chequeo Direcciones disponibles
      if (chequear_esquina(dirCoord)) {
        //Si este direccion es más óptima que continuar recto
        if (maze[dirCoord.y][dirCoord.x].distancia < sig_menor_val) {
          //actualiza el sig menor valor disponible
          sig_menor_val = maze[dirCoord.y][dirCoord.x].distancia;
          //actualiza el valor de sig menor a la direccion
          sig_menor = dirCoord;
          sig_dir = dir;
          Serial.print("Siguente direccion posible del for: ");
          Serial.println(sig_dir);
        }
      }
    }
  }
  Serial.print("Cardinal actual: ");
  Serial.print(cardinal);
  Serial.print("\n");
  return sig_dir;
}



/*
INPUT: coordenada y byte con la informacion de las paredes actuales
OUTPUT: Actualiza el maze con el valor correspondiente segun si hay pared
*/

void actualizar_coord(coord coordenada, byte pared) {
  if (chequear_esquina(coordenada)) {
    if ((maze[coordenada.y][coordenada.x].walls & pared) != 0) {
      maze[coordenada.y][coordenada.x].walls = maze[coordenada.y][coordenada.x].walls - pared;
    }
  }
}


// ##### Flood Fill ####

void flood_fill(coord destino[], coord actual, bool movimiento) {
  coord coord_actual = actual;
  byte cardinal = cardinalGlobal;
  byte sig_cardinal;
  /* Norte / Sur / Este / Oeste
  * 1 = N
  * 4 = E
  * 2 = S
  * 8 = W
  */
  //Mientras no este en la meta !(0) actualizar el maze y moverse a las casillas sig
  while (maze[coord_actual.y][coord_actual.x].distancia != 0) {
    floodFillUpdate(coord_actual, destino, movimiento);
    sig_cardinal = orientar(coord_actual, cardinal);
    Serial.print("decision posible: ");
    Serial.println(sig_cardinal);
    coord sig_coordenada = coordVecina(coord_actual, sig_cardinal);
    Serial.print("pos x: ");
    Serial.print(sig_coordenada.x);
    Serial.println();
    Serial.print("pos y: ");
    Serial.print(sig_coordenada.y);
    Serial.println();

    if (movimiento) {
      //crear_instrucciones(actual, sig_cardinal);

      navegacion.push(crear_movimiento(coord_actual, sig_coordenada, sig_cardinal));
      
      // pop de la instrccion creada2
      
      ejecutar_instrucciones(navegacion.pop());
       
    }
    //Actualizar el valor para las siguientes entradas del loop
    cardinal = sig_cardinal;
    coord_actual = sig_coordenada;
    if (movimiento) {
      cardinalGlobal = cardinal;
      globalCoord = coord_actual;
    }
    printMazewalls();
    printMazedistancia();
    Serial.print("Valor cardinal: ");
    Serial.println(cardinalGlobal);
    Serial.print("Valor coord actual x: ");
    Serial.println(coord_actual.x);
    Serial.print("Valor coord actual y: ");
    Serial.println(coord_actual.y);
  }
  //colocar el globalend como la posicion actual.
  globalEnd = coord_actual;
}

boolean terminado(coord coordenada){
  coord meta[] = { { 2,2},{ 2,3 },{ 3,2 },{ 3,3 } };
  boolean fin = false;
  for (int i = 0; i<4; i++) {
    coord sig = meta[i];
    if (chequear_esquina(coordenada)) {
      if ((coordenada.x == sig.x) && (coordenada.y == sig.y)) {
        fin = true;
      }
    }
  }
  return fin;
}


/*
INPUT: Coord
OUTPUT: un entero dice la distancia menor de todos los vecinos disponibles
*/
int chequear_vecinos(coord coordenada) {
  int minVal = X*Y;
  for (int i = 0; i<sizeof(cardinales); i++) {
    byte dir = cardinales[i];
    //si la direccion es accesible 
    if ((maze[coordenada.y][coordenada.x].walls & dir) != 0) {
      //Obetener la coordenada de la direccion
      coord coordenada_vecina = coordVecina(coordenada, dir);
      //Chequeamos que la coordenada esta dentro del tamano del maze
      if (chequear_esquina(coordenada_vecina)) {
        //si el vecino es uno menos que el actual, almacenar el min
        //Si el minimo es muy grande, probar otro camino
        if (maze[coordenada_vecina.y][coordenada_vecina.x].distancia < minVal) { 
          minVal = maze[coordenada_vecina.y][coordenada_vecina.x].distancia; 
        }
      }
    }
  }
  return minVal;
}


/*
INPUT: Coordenada actual del mouse
OUTPUT: Maze actualizado con las paredes
*/

void floodFillUpdate(coord actual, coord destino[], bool movimiento) {
  StackList<coord> coordenadas;
  if(movimiento){
    maze[actual.y][actual.x].walls = Haypared();
  }
  //maze[actual.y][actual.x].visitado = 1;
  coordenadas.push(actual); 
  for (int i = 0; i<sizeof(cardinales); i++) {
    byte dir = cardinales[i];
    //Si no hay pared en esta coordenada
    if ((maze[actual.y][actual.x].walls & dir) == 0) {
      //Coordenada temporal
      coord workingCoord = { actual.x,actual.y };
      switch (dir) {
      case 1:
        workingCoord.y = workingCoord.y - 1;
        actualizar_coord(workingCoord, 2);
        break;
      case 2:
        workingCoord.y = workingCoord.y + 1;
        actualizar_coord(workingCoord, 1);
        break;
      case 4:
        workingCoord.x = workingCoord.x + 1;
        actualizar_coord(workingCoord, 8);
        break;
      case 8:
        workingCoord.x = workingCoord.x - 1;
        actualizar_coord(workingCoord, 4);
        break;
      }
      //Si la celda tiene una coordenada valida y la coordenada no es la meta push en la pila
      if (chequear_esquina(workingCoord) && (!terminado(workingCoord))) {
        coordenadas.push(workingCoord);
        Serial.println("Coordenada del workingCoord");
        Serial.print(workingCoord.x);
        Serial.print(" ");
        Serial.print(workingCoord.y);
        Serial.println("\n");
      }
    }
  }
  //Mientras la pila no este vacia (tenga coordenadas por actualizar)
  while (!coordenadas.isEmpty()) {
    //Remuevo el ultimo elemento de la pila
    coord celda = coordenadas.pop();
    int vecino_chequear = chequear_vecinos(celda);
    //Regla del flood fill
    if (vecino_chequear + 1 != maze[celda.y][celda.x].distancia) {
      maze[celda.y][celda.x].distancia = vecino_chequear + 1;
      for (int i = 0; i<sizeof(cardinales); i++) {
        byte dir = cardinales[i];
        if ((maze[celda.y][celda.x].walls & dir) != 0) {
          coord sig_coord = coordVecina(celda, dir);
          if (chequear_esquina(sig_coord)) {
            if (!terminado(sig_coord)) {
              coordenadas.push(sig_coord);
            }
          }
        }
      }
    }
  }
}


void retorno(coord coordenada, coord global){
  for(int j = 0; j<Y; j++){
    for(int i = 0; i<X; i++){
      maze[j][i].distancia = calcDist(i, j, coordenada.x, coordenada.y);
    }
  }
  //Entrada por Sur/Este
  if ( (global.x==(X/2))&& (global.y==(Y/2)) ){
    maze[global.x][global.y-1].distancia = maze[global.x][global.y].distancia +1;
    maze[global.x-1][global.y].distancia = maze[global.x][global.y].distancia +1;
    maze[global.x-1][global.y-1].distancia = maze[global.x][global.y].distancia +2;
  }

  //Entrada por Oeste/Este
  if ( (global.x==(X/2)-1)&& (global.y==(Y/2)) ){
    maze[global.x][global.y-1].distancia = maze[global.x][global.y].distancia +1;
    maze[global.x+1][global.y].distancia = maze[global.x][global.y].distancia +1;
    maze[global.x+1][global.y-1].distancia = maze[global.x][global.y].distancia +2;
  }
  //entrada por Sur/norte
  if ( (global.x==(X/2))&& (global.y==(Y/2)-1) ){
    maze[global.x][global.y+1].distancia = maze[global.x][global.y].distancia +1;
    maze[global.x-1][global.y].distancia = maze[global.x][global.y].distancia +1;
    maze[global.x-1][global.y+1].distancia = maze[global.x][global.y].distancia +2;
  }

  //entrada por Oeste / Norte
  if ( (global.x==(X/2)-1)&& (global.y==(Y/2)-1) ){
    maze[global.x][global.y+1].distancia = maze[global.x][global.y].distancia +1;
    maze[global.x+1][global.y].distancia = maze[global.x][global.y].distancia +1;
    maze[global.x+1][global.y+1].distancia = maze[global.x][global.y].distancia +2;
  }
  
        
}


//####### FIN FLOOD FILL ##########

///// ##### REFLOOD



//######## Procedimiento Reflood ########
void reflood(coord meta[]){                                                          /////////////////////TERMINAR
  //Colocar distancia en inicial pero ahora el maze tiene las paredes almacenadas
  reiniciar_maze();
  
  //Run flood fill but without actual motion
  coord actual = {0,0};
  flood_fill(meta, actual, false);
  
  //El mouse permanece en el inicio del maze pero con la distancia mas optima
  
  //creamos cola de velocidad
  //crear_cola_velocidad();
  
  //Ejecutar instrucciones
  //ejecutar_instrcciones();
  
  
}

//Reiniciar distancias
void reiniciar_maze(){
  for(int j = 0; j<Y; j++){
    for(int i = 0; i<X; i++){
      maze[j][i].distancia = calcCentro(i, j, X);
    }
  }
}


void loop() {
  //printMazedistancia();
  coord meta[] = { { (int)X/2,(int)Y/2},{ (int)X/2,(int)Y/2 + 1 },{ (int)X/2 + 1, (int)Y/2 },{(int)X/2 + 1,(int)Y/2 + 1  } };       ////////////////////////////MODIFICAR PARA OTROS SIZES
  //Flood fill
  flood_fill(meta, globalCoord, true);
  Serial.println("Llegue al centro, Inicio Retorno");
 //Nuevo punto a retorno ( Inicio)
  coord inicio[] = {{0,0}};
  retorno(inicio[0], globalCoord);
  
  //printMazedistancia();
  flood_fill(inicio, globalCoord, true);
  Serial.println("Termino fase de exploracion");
  //reflood(meta);  
  flood_fill(meta, globalCoord, true);
  Serial.println("Comienza reflood");
  
  while(1);
}

/////////////////////////////////////EDITAR/////////////////////////
void girar(float angulo){
  
  if (angulo== -90){
    girarAngulo(0, 500);

  }
  else if (angulo== 90){
    girarAngulo(1, 500);

  }
  else if (angulo == 180){
    girarAngulo(1, 250);
    girarAngulo(1, 250);

  }
  
}


instruction crear_movimiento(coord coordActual, coord sig_coord, byte sig_cardinal){
  int casillas = 0;
  float giro =0;
  switch(sig_cardinal){
    case 1:
      if(cardinalGlobal==4){
        giro = -90.0;
      }
      if(cardinalGlobal==8){
        giro = 90.0;
      }
      if(cardinalGlobal==2){
        giro = 180.0;
      }
      break;
    case 2:
      if(cardinalGlobal==4){
        giro = 90.0;
      }
      if(cardinalGlobal==8){
        giro = -90.0;
      }
      if(cardinalGlobal==1){
        giro = 180.0;
      }
      break;
    case 4:
      if(cardinalGlobal==1){
        giro = 90.0;
      }
      if(cardinalGlobal==2){
        giro = -90.0;
      }
      if(cardinalGlobal==8){
        giro = 180.0;
      }
      break;
    case 8:
      if(cardinalGlobal==1){
        giro = -90.0;
      }
      if(cardinalGlobal==2){
        giro = 90.0;
      }
      if(cardinalGlobal==4){
        giro = 180.0;
      }
      break;
  }
  
  if(coordActual.x != sig_coord.x){
    casillas = abs(coordActual.x - sig_coord.x);
  }
  
  else if(coordActual.y != sig_coord.y){
    casillas = abs(coordActual.y - sig_coord.y);
  }

  instruction movimiento;
  movimiento.casillas = casillas;
  movimiento.giro = giro;
  return movimiento;
}

void ejecutar_instrucciones(instruction mov){
  girar(mov.giro);
  //delay(3000);
  fowardNPasos(mov.casillas, 300);
  //delay(3000);
}
