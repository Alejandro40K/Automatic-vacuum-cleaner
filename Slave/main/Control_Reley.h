///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  PROYECTO MODULAR ASPIRADORA AUTOMATA USO RUDO
//
// Editores:           Alejando Orozco Romo
// Archivo:            Control_Reley.h
// Fecha de edicion:   25/04/2025
// Notas: 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef CONTROL_RELEY_H
#define CONTROL_RELEY_H

// BIBLIOTECAS
#include <Arduino.h>
#include <PCF8574.h>
#include <Wire.h>

// PINES DEL PCF8574
#define PCF8574_ADDR 0x20
#define RELE_TURBINA_SUCCION 0
#define RELE_BOMBA_AGUA 1  

//VARIABLES EXTERNAS
extern PCF8574 pcf8574;

//FUNCIONES PARA EL CONTROL DE LOS RELÉS
void iniciarRelevadores();
void comenzarRelevadores();

void activarTurbinaSuccion();
void desactivarTurbinaSuccion();

void activarBombaAgua();
void desactivarBombaAgua();

#endif


