///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  PROYECTO MODULAR ASPIRADORA AUTOMATA USO RUDO
//
// Editores:           Alejando Orozco Romo
// Archivo:            Control_Relcoleccion.h
// Fecha de edicion:   25/04/2025
// Notas: 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef CONTROL_RECOLECCION_H
#define CONTROL_RECOLECCION_H

// BIBLIOTECAS
#include <Arduino.h>

// PINES DEL ESP32
#define MOTOR_SUCCION 4
#define BOMBA_AGUA 16
#define CEPILLO_CENTRAL 17
#define CEPILLO_IZQUIERDO 5
#define CEPILLO_DERECHO 18

//FUNCIONES PARA EL SISTEMA DE RECOLECCION
void activarCepilloCentral();
void desactivarCepilloCentral();

void activarCepilloIzquierdo();
void desactivarCepilloIzquierdo();

void activarCepilloDerecho();
void desactivarCepilloDerecho();

void activarBombaAgua();
void desactivarBombaAgua();

void activarMotorSuccion();
void desactivarMotorSuccion();

void activarActuadores();
void desactivarActuadores();
void iniciarRecoleccion();

#endif


