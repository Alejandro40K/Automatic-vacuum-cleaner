///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  PROYECTO MODULAR ASPIRADORA AUTOMATA USO RUDO
//
// Editores:           Alejando Orozco Romo
// Archivo:            Control_Aspersion.h
// Fecha de edicion:   25/04/2025
// Descripcion:        

// Notas: 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef SISTEMA_ASPERSION_H
#define SISTEMA_ASPERSION_H

// BIBLIOTECAS
#include <Arduino.h>
#include <Wire.h>
#include <DHT.h> 
// PINES DEL ESP32 (POR DEFINIR LOS DEL ULTRASONICO)
#define TRIGGER_PIN 23
#define ECHO_PIN 19

void iniciarAspersion();
void medirNivelTanque();
void actualizarNivelTanque();
void controlarSistemaAspersion();
void actualizarAspersion();


#endif


