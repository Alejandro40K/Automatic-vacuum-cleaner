///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  PROYECTO MODULAR ASPIRADORA AUTOMATA USO RUDO
//
// Editores:           Alejando Orozco Romo
// Archivo:            Control_Motores.h
// Fecha de edicion:   25/05/2025
// Notas: 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef CONTROL_MOTORES_H
#define CONTROL_MOTORES_H

// BIBLIOTECAS
#include <Arduino.h>

// PINES DEL ESP32 

// PUENTE H UNO
#define RPWM_A 13
#define LPWM_A 12
#define R_EN_A 14
#define L_EN_A 27

// PUENTE H DOS
#define RPWM_B 36
#define LPWM_B 39
#define R_EN_B 34
#define L_EN_B 35


// FUNCIONES PARA EL CONTROL DE LOS MOTORES
void iniciarMotores();
void comenzarMotores();

void moverMotorA(int velocidad);
void moverMotorB(int velocidad);

#endif


