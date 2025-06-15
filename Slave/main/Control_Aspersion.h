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

//VARIABLES EXTERNAS
// varriables externas sensor ultrasonico
extern long tiempo;
extern int distancia;
extern int nueva_distancia;
extern float velocidad;

//FUNCIONES PARA CONTROLAR LA BOMBA DEL ASPERSOR
void controlarSistemaAspersion();

//FUNCIONES PARA EL SENSOR ULTRASÓNICO
void leerNivelTanque();
void iniciarUltrasonico();

//FUNCION DE PRUEBA
void comenzarAspersion();
void iniciarAspersion();


#endif


