///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  PROYECTO MODULAR ASPIRADORA AUTOMATA USO RUDO
//
// Editores:           Alejando Orozco Romo
// Archivo:            Control_Aspersion.cpp
// Fecha de edicion:   25/04/2025
// Descripcion:        

// Notas: 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//******************************************************************BIBLIOTECAS**********************************************************************//
#include "Control_Aspersion.h"
#include "Control_Recoleccion.h"

//**************************************************************DEFINIMOS VARIABLES******************************************************************//
long tiempo;
int distancia;
int nueva_distancia = 0;
float velocidad = 0.01715; 
// Modificar las dimenciones a partir del tanque de agua (en cm)
const float distanciaMaxima = 10.0;  
const float distanciaMinima = 0.0; 
float porcentajeLleno = 100;  

//*********************************************************FUNCIONES PARA EL CONTROL DE ASPERSION****************************************************//

// FUNCIONES DE INICIO Y PRUEBA 
void iniciarAspersion(){
    iniciarUltrasonico();
}

void comenzarAspersion(){
  leerNivelTanque();
  controlarSistemaAspersion();
}

void controlarSistemaAspersion(){
    if (porcentajeLleno < 10) {
        desactivarBombaAgua();
        Serial.println("Aspersor Apagado");
    } 
    else if (porcentajeLleno > 10) {
        Serial.println("Aspersor Activado por 3 segundos");
        activarBombaAgua();
        delay(3000);  
        desactivarBombaAgua();
        Serial.println("Esperando 10 segundos antes de volver a medir");
        delay(20000); 
    }
}

// FUNCIONES SENSOR ULTRASONICO (MIDE CAPACIDAD DEL DEPOSITO DE AGUA)
void iniciarUltrasonico(){
  pinMode(TRIGGER_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT);
}

void leerNivelTanque(){
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  tiempo = pulseIn(ECHO_PIN, HIGH);
  distancia = (tiempo * velocidad);  
  porcentajeLleno = map(distancia, distanciaMaxima, distanciaMinima, 0, 100);  
  if (porcentajeLleno < 0) porcentajeLleno = 0;
  if (porcentajeLleno > 100) porcentajeLleno = 100;

  Serial.println("Nivel del tanque: ");
  Serial.println(porcentajeLleno);

  delay(500);  
}
