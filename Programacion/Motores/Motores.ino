#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"

////////////////////////////// VARIABLES MOTORES ///////////////////////////////////////
#define GPIO_PF2_M1PWM6         0x00050805
#define GPIO_PF3_M1PWM7         0x00050C05
#define GPIO_PA6_M1PWM2         0x00001805
#define GPIO_PA7_M1PWM3         0x00001C05
#define NSLEEP PC_7
#define FAUTL PD_7
#define FORWARD 1
#define BACKWARD 0

/////////////////////////////////////////////////////////////////////////////////////////

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
//--------------------------------------------------------------------------------------

//****************************** SETUP MOTORES ****************************************
void PWM() {
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

//*************************************************************************************
void setup() {
  // put your setup code here, to run once:
  PWM();
}

void loop() {
 PWM_D(500,FORWARD);
 PWM_I(500,FORWARD);
 delay(3000);
 PWM_D(0,FORWARD);
 PWM_I(0,FORWARD);
 while(1);
}
