///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  PROYECTO MODULAR ASPIRADORA AUTOMATA USO RUDO
//
// Editores:           Alejando Orozco Romo
// Archivo:            Control_Reley.cpp
// Fecha de edicion:   25/04/2025
// Descripcion:       
// Notas: 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//******************************************************************BIBLIOTECAS**********************************************************************//
#include "Control_Reley.h"

//****************************************************************DEFINIMOS OBJETOS******************************************************************//
PCF8574 pcf8574(PCF8574_ADDR);

//**************************************************************FUNCIONES DE CONTROL*****************************************************************//

// FUNCIONES DE INICIO Y PRUEBA 
void iniciarRelevadores(){
  Wire.begin(21, 22);
  pcf8574.begin();
  pcf8574.write(RELE_TURBINA_SUCCION, HIGH);
  pcf8574.write(RELE_BOMBA_AGUA, HIGH);

};

void comenzarRelevadores(){
  activarTurbinaSuccion();
  activarBombaAgua();
  delay(5000);
  desactivarTurbinaSuccion();
  desactivarBombaAgua();
  delay(5000);
};

// FUNCIONES PARA LA TURBINA DE SUCCION 
void activarTurbinaSuccion(){
  pcf8574.write(RELE_TURBINA_SUCCION, LOW);
  //Serial.print("turbina activado\n");

};

void desactivarTurbinaSuccion(){
  pcf8574.write(RELE_TURBINA_SUCCION, HIGH);
  //Serial.print("turbina desactivado\n");
};

// FUNCIONES PARA LA BOMBA DE AGUA 
void activarBombaAgua(){
  pcf8574.write(RELE_BOMBA_AGUA, LOW);
  //Serial.print("bomba activado\n");

};

void desactivarBombaAgua(){
  pcf8574.write(RELE_BOMBA_AGUA, HIGH);
  //Serial.print("turbina desactivado\n");
};
