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

int calibracionFD= 1569;         //Constante definida para probar alinear
int calibracionLD = 1248;         // Calibracion lateral derecha
int calibracionLI = 1327;         // Calibracion lateral izquierda
int calibracionFI= 1230;         //Constante definida para probar alinear
int calibracionF=0;          //Constante calibracion frontal

////////////////////////////Variables de paredes ////////////////////////////
int Pared_D;
int Pared_I;
int Pared_De;
int Pared_DiD;
int Pared_DiI;
///////////////////////// Otras Variables//////////////////////////////
int LED1 = PA_2;
int LED2 = PA_3;
int inicio=0;

void setup() {
  Serial.begin(115200);
  LEDS();   
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
}

void loop() {
  ////////Calibracion Constantes Pared //////////////////////////////////////////
  int dato=(medir(6));
  
  enviarSerial(dato);
  if (dato>1000)
    digitalWrite(LED1,HIGH);
  else
    digitalWrite(LED1,LOW);
  delay(500);
}

void LEDS(){
  analogReadResolution(12);// ajuste ADC en 12 bits
  pinMode(emisor1, OUTPUT); 
  pinMode(emisor3, OUTPUT);
  pinMode(emisor4, OUTPUT);
  pinMode(emisor6, OUTPUT);
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

void enviarSerial(int dato){
  char buffer[5];
  for(int i=0;i<4;i++){
    buffer[i]=dato>>i*8;  
  }
  buffer[4]=0xFF;
  for(int i=0;i<5;i++){
    Serial.write(buffer[i]);
  }
}
