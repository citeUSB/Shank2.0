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
int emisor2=PB_5;
int emisor3=PD_6;
int emisor4=PD_0;
int emisor5=PE_5;
int emisor6=PE_4;

//receptores
int receptor1=PD_1;
int receptor2=PD_2;
int receptor3=PD_3;
int receptor4=PE_2;
int receptor5=PE_1;
int receptor6=PE_0;

int cal_FD=0;    //Constante definida para probar alinear
int cal_DiD=0;        // Calibracion diagonal derecha
int cal_LD=0;         // Calibracion lateral derecha
int cal_LI=0;         // Calibracion lateral izquierda
int cal_DiI=0;        // Calibracion diagonal izquierda
int cal_FI=0;    //Constante definida para probar alinear
int media_D=0;

////////////////////////////Variables de paredes ////////////////////////////
int Pared_D;
int Pared_I;
int Pared_De;
int Pared_DiD;
int Pared_DiI;
///////////////////////// Otras Variables//////////////////////////////
int i;
int dato;
int LED1 = PA_2;
int LED2 = PA_3;
int inicio=0;

void setup() {
  Serial.begin(9600);
  LEDS();
  //paredes 
  Pared_D = cal_LD;   // 1300  
  /*VALOR PEGADO PARED DERECHA: 3912*/
  Pared_I = cal_LI; // 2700
  Pared_De=1200;                      //media_D-2800;  //2800
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
}

void loop() {
  ////////Calibracion Constantes Pared Lateral Derecha//////////////////////////////////////////
  dato=Medir(1);
  send_serial(dato);
  if (dato>1000)
    digitalWrite(LED1,HIGH);
  else
    digitalWrite(LED1,LOW);

  delay(500);

/*
/////////////Calibracion Constantes Parde Laterial Izquierda/////////////
  dato=Medir(4);
  send_serial(dato);
  if (dato>Pared_I)
    digitalWrite(LED2,HIGH);
  else
    digitalWrite(LED2,LOW);
*/

/*
//////////////////Calibracion Constantes Pared Delantero////////////
  dato=(Medir(1)+Medir(6))/2;
  send_serial(dato);
  if (dato>Pared_De)
    digitalWrite(LED2,HIGH);
  else
    digitalWrite(LED2,LOW);
/*
 * 
////////// medida de referencia para que detecte pared delantera sin cruzar///////
  dato=Medir(6);
  send_serial(dato);
 */ 
}

void LEDS(){
  analogReadResolution(12);// ajuste ADC en 12 bits
  pinMode(emisor1, OUTPUT); 
  pinMode(emisor2, OUTPUT);
  pinMode(emisor3, OUTPUT);
  pinMode(emisor4, OUTPUT);
  pinMode(emisor5, OUTPUT);
  pinMode(emisor6, OUTPUT);
  cal_FD = Medir(1);
  cal_LD = Medir(3);
  cal_LI = Medir(4);
  cal_FI = Medir(6);
  cal_DiD = Medir(2);
  cal_DiI = Medir(5);
  media_D = (cal_FD+cal_FI)/2;
  
}

int Medir(int n){
  int aux=0;
  int E, R;
  int veces=1;
  int i=0;
  int r_on = 0;  // medida con los led prendidos
  int r_off = 0; // medida con los led apagados
  switch(n){
    case 1: E=emisor1; R=receptor1; break; 
    case 2: E=emisor2; R=receptor2; break;
    case 3: E=emisor3; R=receptor3; break;
    case 4: E=emisor4; R=receptor4; break;
    case 5: E=emisor5; R=receptor5; break;
    case 6: E=emisor6; R=receptor6; break; 
  }
  for (i=0;i<veces;i++){
    digitalWrite(E,HIGH);// encendido emisor
    delayMicroseconds(40);
    r_on = analogRead(R);  // lectura ADC
    digitalWrite(E,LOW);    // apagado emisor
    delayMicroseconds(100);
    r_off = analogRead(R); // lectura ADC 
    aux= aux + r_on;
  }
  return(aux/veces);
}

void send_serial(int dato){
  byte buffer[5];
  for (i=0;i<4;i++){
    buffer[i]=dato>>i*8;  }
  buffer[4]=0xFF;
  Serial.print("DATOS:   ");
  Serial.println(dato);/*
  for (i=0;i<5;i++){
    Serial.write(buffer[i]);
  }*/
}
