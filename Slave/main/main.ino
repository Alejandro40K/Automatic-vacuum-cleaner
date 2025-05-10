///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  PROYECTO MODULAR ASPIRADORA AUTOMATA USO RUDO
//
// Editores:           Alejando Orozco Romo
// Archivo:            Slave / main.ino
// Fecha de edicion:   24/04/2025
// Descripcion:         
// Notas: 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Control_Recoleccion.h"
#include "Control_Reley.h"
#include "Control_Aspersion.h"
#include "Control_Motores.h"

#define DIR_IN1 2
#define DIR_IN2 15

uint8_t last_dir = 255;

void setup() {
  Serial.begin(115200);
  pinMode(DIR_IN1, INPUT);
  pinMode(DIR_IN2, INPUT);

  // inicialización de tus sistemas aquí
}

void loop() {
  procesarDireccion();
}

// Interpreta el comando de dirección en 2 bits
void procesarDireccion() {
  uint8_t dir = (digitalRead(DIR_IN1) << 1) | digitalRead(DIR_IN2);
  
  if (dir != last_dir) {
    Serial.print("Dirección recibida: ");
    Serial.println(dir);

    if (dir == 3) {
      // forward
  
    } else if (dir == 2) {
      // left
     
    } else if (dir == 1) {
      // right
      
    } else if (dir == 0) {
      // stop
    }

    last_dir = dir;
  }
}
