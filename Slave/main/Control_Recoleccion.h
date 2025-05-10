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
#define CEPILLO_CENTRAL 5
#define CEPILLO_IZQUIERDO 18
#define CEPILLO_DERECHO 19

//FUNCIONES PARA EL SISTEMA DE RECOLECCION
void activarCepilloCentral();
void desactivarCepilloCentral();

void activarCepilloIzquierdo();
void desactivarCepilloIzquierdo();

void activarCepilloDerecho();
void desactivarCepilloDerecho();

void iniciarRecoleccion();
void comenzarRecoleccion();

#endif


