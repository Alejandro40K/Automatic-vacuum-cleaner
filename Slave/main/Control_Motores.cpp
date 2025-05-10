///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  PROYECTO MODULAR ASPIRADORA AUTOMATA USO RUDO
//
// Editores:           Alejando Orozco Romo
// Archivo:            Control_Motores.cpp
// Fecha de edicion:   25/04/2025
// Descripcion:       
// Notas: 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//******************************************************************BIBLIOTECAS**********************************************************************//
#include "Control_Motores.h"

//**************************************************************DEFINIMOS VARIABLES******************************************************************//
const int pwm_freq = 5000;
const int pwm_resolution = 8;
const int velocidad = 180;

//*******************************************************FUNCIONES PARA EL CONTROL DE MOTORES********************************************************//

// FUNCIONES INICIO Y PRUEBA
void iniciarMotores(){
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
};

void comenzarMotores(){
  //Serial.println("Avance...");
  ledcWrite(LPWM_A, velocidad);  // PWM a pin LPWM
  ledcWrite(RPWM_A, 0);
  delay(3000);

  //Serial.println("Detenido...");
  ledcWrite(LPWM_A, 0);
  ledcWrite(RPWM_A, 0);
  delay(1000);

  //Serial.println("Reversa...");
  ledcWrite(LPWM_A, 0);
  ledcWrite(RPWM_A, velocidad);  // PWM a pin RPWM
  delay(3000);

  //Serial.println("Detenido...");
  ledcWrite(LPWM_A, 0);
  ledcWrite(RPWM_A, 0);
  delay(1000);
};

//*********************************************************FUNCIONES PARA EL CONTROL DE LOS MOTORES**************************************************//

void moverMotorA(int velocidad) {
  if (velocidad > 0) {
    ledcWrite(LPWM_A, velocidad);    // PWM adelante
    ledcWrite(RPWM_A, 0);            // Reversa apagada
  } else if (velocidad < 0) {
    ledcWrite(LPWM_A, 0);
    ledcWrite(RPWM_A, -velocidad);   // PWM reversa
  } else {
    ledcWrite(LPWM_A, 0);
    ledcWrite(RPWM_A, 0);            // Motor detenido
  }
}

void moverMotorB(int velocidad) {
  if (velocidad > 0) {
    ledcWrite(LPWM_B, velocidad);
    ledcWrite(RPWM_B, 0);
  } else if (velocidad < 0) {
    ledcWrite(LPWM_B, 0);
    ledcWrite(RPWM_B, -velocidad);
  } else {
    ledcWrite(LPWM_B, 0);
    ledcWrite(RPWM_B, 0);
  }
}
