///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  PROYECTO MODULAR ASPIRADORA AUTOMATA USO RUDO
//
// Editores:           Alejando Orozco Romo
// Archivo:            Slave / main.ino
// Fecha de edicion:   24/04/2025
// Descripcion:         
// Notas:  ESTA FUNCIONABA POR COMUNICACIO DE DOS BIT Y POR MEDIO DE FLECHAS, EN ESTE MOMENTO YA NO CONVIENE TENER UNA FSM, CANVIARLA ,
// revisar chgtp, sustituir este codigo por todo el control de motores original por aqui, y simplemente implementar una logica sensilla para ahorar
// tiempo. pasat TAL CUAL EL CODIGO ORIGINAL . 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//******************************************************************BIBLIOTECAS**********************************************************************//
#include "motorControl.h"
#include "Control_Aspersion.h"
#include "Control_Recoleccion.h"

//******************************************************************PINS ESP32**********************************************************************//
// MOTOR DERECHO
#define C1R 39
#define C2R 36

// PUENTE H DOS
#define RPWM_B 32
#define LPWM_B 33
#define R_EN_B 25
#define L_EN_B 26
<
// MOTOR IZQUIERDO
#define C1L 35
#define C2L 34

// PUENTE H UNO
#define RPWM_A 13
#define LPWM_A 12
#define R_EN_A 14
#define L_EN_A 27

// COMUNICACION UART 
#define UART_RX 2   
#define UART_TX 15  
#define UART_BAUD 115200

//******************************************************************TIEMPO DE MUESTREO**********************************************************************//
unsigned long lastTime = 0, sampleTime = 100;

//******************************************************************MOTOR DERECHO**********************************************************************//
motorControl motorR(sampleTime);

// Encoder
volatile int nR = 0;
volatile int antR      = 0;
volatile int actR      = 0;

int channelMotorR = 0;

// Variables
int cvR = 0;
double wR = 0;
double wRref = 0;

//******************************************************************MOTOR IZQUIERDO**********************************************************************//
motorControl motorL(sampleTime);
// Encoder
volatile int nL = 0;
volatile int antL      = 0;
volatile int actL      = 0; 

int channelMotorL = 1;

// Variables
int cvL = 0;
double wL = 0;
double wLref = 0;

//**************************************************************VARIABLES PARA CALCULAR VELOCIDADES ANGULARES******************************************************************//

double constValue = 4.1975; // (1000*2*pi)/R ---> R = 1496.88 - 350RPM Resolucion encoder cuadruple

const int pwm_freq = 5000;
const int pwm_resolution = 8;
const int velocidadFija  = 180;

//*******************************************************ROBOT********************************************************//
double uMeas  = 0;
double wMeas  = 0;
double xp = 0.0, yp = 0.0;
double x = 0.0, y = 0.0;
double phi = 0.0;

const double R = 0.0335; // radio de la llanta
const double d = 0.187; // Distancia entre llantas

const double umbralMovimiento = 0.01; // m/s

//*******************************************************BATERIA********************************************************//
//const int pinB = 15;
//const int pinZ = 4;
//double voltaje = 0;

//*******************************************************COMUNICACION UART********************************************************//
double v = 0.0;  // velocidad lineal (m/s)
double w = 0.0;  // velocidad angular (rad/s)

String inputString = "";  // cadena para almacenar la línea recibida
bool stringComplete = false;

void comunicacionUart() {
  // Leer caracteres desde UART
  while (Serial2.available()) {
    char inChar = (char)Serial2.read();
    if (inChar == '\n') {  // fin de mensaje
      stringComplete = true;
      break;
    } else {
      inputString += inChar;
    }
  }

  // Si llegó un mensaje completo
  if (stringComplete) {
    inputString.trim();  // elimina espacios y saltos de línea

    // Parsear v y w desde la cadena "v,w"
    int commaIndex = inputString.indexOf(',');
    if (commaIndex > 0) {
      v = inputString.substring(0, commaIndex).toFloat();
      w = inputString.substring(commaIndex + 1).toFloat();
      velocityMotor(v, w);

      // Imprimir para confirmar recepción
      Serial.print("v = ");
      Serial.print(v);
      Serial.print("  |  w = ");
      Serial.println(w);
    }

    // Limpiar para el siguiente mensaje
    inputString = "";
    stringComplete = false;
  }
}


//*******************************************************INTERRUPCIONES********************************************************//
void IRAM_ATTR encoderR()
{
  antR=actR;
               
  if(digitalRead(C2R)) bitSet(actR,0); else bitClear(actR,0);
  if(digitalRead(C1R)) bitSet(actR,1); else bitClear(actR,1);
  
  
  if(antR == 2 && actR ==0) nR--;
  if(antR == 0 && actR ==1) nR--;
  if(antR == 3 && actR ==2) nR--;
  if(antR == 1 && actR ==3) nR--;
  
  if(antR == 1 && actR ==0) nR++;
  if(antR == 3 && actR ==1) nR++;
  if(antR == 0 && actR ==2) nR++;
  if(antR == 2 && actR ==3) nR++;    

}

void IRAM_ATTR encoderL()
{
  antL=actL;
               
  if(digitalRead(C2L)) bitSet(actL,0); else bitClear(actL,0);
  if(digitalRead(C1L)) bitSet(actL,1); else bitClear(actL,1);
  
  
  if(antL == 2 && actL ==0) nL++;
  if(antL == 0 && actL ==1) nL++;
  if(antL == 3 && actL ==2) nL++;
  if(antL == 1 && actL ==3) nL++;
  
  if(antL == 1 && actL ==0) nL--;
  if(antL == 3 && actL ==1) nL--;
  if(antL == 0 && actL ==2) nL--;
  if(antL == 2 && actL ==3) nL--;     
}


//*******************************************************FUNCION DE INICIO********************************************************//
void iniciarMotores(){

  Serial.begin(115200);  // monitor para verificar recepción
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);  // UART maestro -> esclavo
  inputString.reserve(50); // memoria para la cadena
  
  // Configuracion de los pines 
  pinMode(C1R, INPUT);
  pinMode(C2R, INPUT);
  pinMode(C1L, INPUT);
  pinMode(C2L, INPUT);

  pinMode(RPWM_A, OUTPUT);
  pinMode(LPWM_A, OUTPUT);
  pinMode(R_EN_A, OUTPUT);
  pinMode(L_EN_A, OUTPUT);

  pinMode(RPWM_B, OUTPUT);
  pinMode(LPWM_B, OUTPUT);
  pinMode(R_EN_B, OUTPUT);
  pinMode(L_EN_B, OUTPUT);

  // Habilitamos motores 
  digitalWrite(L_EN_A, HIGH);
  digitalWrite(R_EN_A, HIGH);
  digitalWrite(L_EN_B, HIGH);
  digitalWrite(R_EN_B, HIGH);

  ledcAttach(LPWM_A, pwm_freq, pwm_resolution);  
  ledcAttach(RPWM_A, pwm_freq, pwm_resolution);  
  ledcAttach(LPWM_B, pwm_freq, pwm_resolution);
  ledcAttach(RPWM_B, pwm_freq, pwm_resolution);

  //parar(RPWM_A,LPWM_A,R_EN_A,L_EN_A);
  //parar(RPWM_B,LPWM_B,R_EN_B,L_EN_B);
  parar(RPWM_A, LPWM_A);
  parar(RPWM_B, LPWM_B);

  // Interrupciones 
  attachInterrupt(C1R, encoderR, CHANGE);
  attachInterrupt(C2R, encoderR, CHANGE);

  attachInterrupt(C1L, encoderL, CHANGE);
  attachInterrupt(C2L, encoderL, CHANGE); 

  ////////////////// Limites de señales //////////////////
  motorR.setCvLimits(255,0);
  motorR.setPvLimits(19,0);

  //motorR.lambdaTunning(0.7902,0.1723,0.1666);
  motorR.setGains(0.63, 0.25, 0.01); // (Kc,Ti,Td)
  
  ////////////////// Limites de señales //////////////////
  motorL.setCvLimits(255,0);
  motorL.setPvLimits(19,0);

  //motorL.lambdaTunning(0.7902,0.1723,0.1666);
  motorL.setGains(0.63, 0.25, 0.01); // (Kc,Ti,Td)
  
  lastTime = millis();


};

//*******************************************************FUNCIONES PRINCIPALES********************************************************//
void setup() {
  iniciarMotores();
  iniciarPerifericos();
}

void loop() {
  comunicacionUart();

  if(millis()-lastTime >= sampleTime)
  {
    
    
    wR = constValue*nR/(millis()-lastTime); // Velocidad angular del lado derecho [rad/s]
    //wR = 0.7*(constValue*nR/(millis()-lastTime)) + 0.3*wR;  //Velocidad derecha con filtro
    wL = constValue*nL/(millis()-lastTime); // Velocidad angular del lado izquierdo [rad/s]
    //wL = 0.7*(constValue*nL/(millis()-lastTime)) + 0.3*wL;  //Velocidad izquierda con filtro
    
    lastTime = millis();
    
    nR = 0;
    nL = 0;
    
    velocityRobot(wR,wL);
    // Calulamos la orientación actual de nuestro robot
    phi = phi + 0.1 * wMeas;
    
    xp = uMeas*cos(phi);
    yp = uMeas*sin(phi);
    
    x = x + 0.1*xp;
    y = y + 0.1*yp; 
      
    // Si se detecta movumiento
    // if (uMeas != 0 || wMeas != 0) 
    if (v != 0 || w != 0){   
        activarActuadores();   
        actualizarAspersion();        
    } else {
        desactivarActuadores();     
    }

    cvR = motorR.compute(wRref,wR); // Control PID
    cvL = motorL.compute(wLref,wL); // Control PID

    if (cvR > 0) giroHorario(RPWM_B, LPWM_B, cvR);
    else if (cvR < 0) giroAntiHorario(RPWM_B, LPWM_B, abs(cvR));
    else parar(RPWM_B, LPWM_B);

    if (cvL > 0) giroHorario(RPWM_A, LPWM_A, cvL);
    else if (cvL < 0) giroAntiHorario(RPWM_A, LPWM_A, abs(cvL));
    else parar(RPWM_A, LPWM_A);

    //battery();
    
    /*SerialBT.println("E");
    SerialBT.println(x);
    SerialBT.println(y);
    SerialBT.println(phi);
    SerialBT.println(xp);
    SerialBT.println(yp);
    SerialBT.println(uMeas);
    SerialBT.println(wMeas);*/
 
  }

}


void parar(int RPWM, int LPWM) {
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);
}

void giroHorario(int RPWM, int LPWM, int pwmValue) {
  ledcWrite(RPWM, pwmValue);  // PWM al pin de avance
  ledcWrite(LPWM, 0);         // apagamos el pin contrario
}

void giroAntiHorario(int RPWM, int LPWM, int pwmValue) {
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, pwmValue);  // PWM al pin de retroceso
}

void velocityMotor(double u, double w)
{

  /*
  Parámetros: Recibe la velocidad lineal y la velocidad angular.
  Descripción: Esta funcion convirte las velocidades globales en las
               velocidades para cada motor, usando las formulas de 
               velocidades dividida entre el radio de la llanta.
  Retonro: Sin retorno.
  */

  wRref = (u+(d*w/2.0))/R; 
  wLref = (u-(d*w/2.0))/R; 

}

void velocityRobot(double w1, double w2)
{

  /*
  Parámetros: Recibe las velocidades angulares de cada motor.
  Descripción: Esta funcion envía las velocidades globales medidas, 
               las cuales nos son útiles para las simulaciones.
               usamos las formulas de velocidades globales.
  Retonro: Sin retorno.
  */

  uMeas = (R*(w1+w2))/2.0;
  wMeas = (R*(w1-w2))/d;
}

//*******************************************************INICIAR PERIFERICOS********************************************************//
void iniciarPerifericos(){
  iniciarAspersion();
  iniciarRecoleccion();

}