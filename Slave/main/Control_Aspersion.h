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
#define DHTPIN 4
#define TRIGGER_PIN 5
#define ECHO_PIN 6
#define DHTTYPE DHT11

//VARIABLES EXTERNAS
// variables sensor DHT11
extern DHT dht; 
extern float humedad;
// varriables externas sensor ultrasonico
extern long tiempo;
extern int distancia;
extern int nueva_distancia;
extern float velocidad;

//FUNCIONES PARA EL SENSOR DE HUMEDAD DHT11
void leerHumedad();

//FUNCIONES PARA CONTROLAR LA BOMBA DEL ASPERSOR
void controlarSistemaAspersion();

//FUNCIONES PARA EL SENSOR ULTRASÓNICO
void leerNivelTanque();
void iniciarUltrasonico();

//FUNCION DE PRUEBA
void comenzarAspersion();
void iniciarAspersion();


#endif


