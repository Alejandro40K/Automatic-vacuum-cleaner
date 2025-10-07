///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  PROYECTO MODULAR ASPIRADORA AUTOMATA USO RUDO
//
// Editores:           Alejando Orozco Romo
// Archivo:            Control_Recoleccion.cpp
// Fecha de edicion:   25/04/2025
// Descripcion:       
// Notas: 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//******************************************************************BIBLIOTECAS**********************************************************************//
#include "Control_Recoleccion.h"

//*********************************************************FUNCIONES PARA SISTEMA DE RECOLECCION*****************************************************//

// FUNCIONES PARA INICIO Y PRUEBA 
void iniciarRecoleccion(){
  
  pinMode(CEPILLO_CENTRAL, OUTPUT);
  pinMode(CEPILLO_IZQUIERDO, OUTPUT);
  pinMode(CEPILLO_DERECHO, OUTPUT);
  pinMode(BOMBA_AGUA, OUTPUT);
  pinMode(MOTOR_SUCCION, OUTPUT);

  digitalWrite(CEPILLO_CENTRAL, LOW);
  digitalWrite(CEPILLO_IZQUIERDO, LOW);
  digitalWrite(CEPILLO_DERECHO, LOW);
  digitalWrite(BOMBA_AGUA, LOW);
  digitalWrite(MOTOR_SUCCION, LOW);


};

// FUNCIONES PARA EL CEPILLO CENTRAL
void activarCepilloCentral(){
  digitalWrite(CEPILLO_CENTRAL, HIGH);
  //Serial.print("cepillo central activado\n");
};

void desactivarCepilloCentral(){
  digitalWrite(CEPILLO_CENTRAL, LOW);
  //Serial.print("cepillo central desactivado\n");
};

// FUNCIONES PARA EL CEPILLO IZQUIERDO
void activarCepilloIzquierdo(){
  digitalWrite(CEPILLO_IZQUIERDO, HIGH);
  //Serial.print("cepillo iz activado\n");

};

void desactivarCepilloIzquierdo(){
  digitalWrite(CEPILLO_IZQUIERDO, LOW);
  //erial.print("cepillo iz desactivado\n");
};

// FUNCIONES PARA EL CEPILLO DERECHO
void activarCepilloDerecho(){
  digitalWrite(CEPILLO_DERECHO, HIGH);
  //Serial.print("cepillo de desactivado\n");

};

void desactivarCepilloDerecho(){
  digitalWrite(CEPILLO_DERECHO, LOW);
  //Serial.print("cepillo de desactivado\n");

};

// FUNCIONES PARA BOMBA DE AGUA 
void activarBombaAgua(){
  digitalWrite(BOMBA_AGUA, HIGH);
};

void desactivarBombaAgua(){
  digitalWrite(BOMBA_AGUA, LOW);
};

// FUNCIONES PARA EL MOTOR DE SUCCION 
void activarMotorSuccion(){
  digitalWrite(MOTOR_SUCCION, HIGH);
};

void desactivarMotorSuccion(){
  digitalWrite(MOTOR_SUCCION, LOW);
};




void activarActuadores(){
  activarCepilloCentral();
  activarCepilloIzquierdo();
  activarCepilloDerecho();
  activarMotorSuccion();

};

void desactivarActuadores(){
  desactivarCepilloCentral();
  desactivarCepilloIzquierdo();
  desactivarCepilloDerecho();
  desactivarMotorSuccion();
  desactivarBombaAgua();
};