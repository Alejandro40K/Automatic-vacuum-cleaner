///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  PROYECTO MODULAR ASPIRADORA AUTOMATA USO RUDO
//
// Editores:           Alejando Orozco Romo
// Archivo:            Slave / Com_UART.cpp
// Fecha de edicion:   25/04/2025
// Descripcion:       
// Notas: 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef SISTEMA_ASPIRADORA_H
#define SISTEMA_ASPIRADORA_H

// BIBLIOTECAS
#include <Arduino.h>
#include <Servo.h>  // Librería para controlar el ESC

// PINES DEL ARDUINO UNO
#define ESC_PIN 13  // PWM para controlar la velocidad del BLDC mediante un ESC (Electronic Speed Controller)
#define MOTOR_CEPILLO_IZQ_A 12 
#define MOTOR_CEPILLO_IZQ_B 11
#define MOTOR_CEPILLO_DER_A 10 
#define MOTOR_CEPILLO_DER_B 9  
#define MOTOR_CEPILLO_CENTRAL_A 8 
#define MOTOR_CEPILLO_CENTRAL_B 7

//VARIABLES EXTERNAS
extern Servo esc;  // Declaración global de la variable esc


//FUNCIONES PARA EL SISTEMA DE RECOLECCION
void activarCepillos();
void detenerCepillos();
void activarSuccion(int potencia);
void desactivarSuccion();
void iniciarSistemaRecoleccion();
void comenzarSistemaRecoleccion();


#endif


