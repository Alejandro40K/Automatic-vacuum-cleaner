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
#include "Control_Aspersion.h"


#define DIR_IN1 2
#define DIR_IN2 15

// Definición de la máquina de estados para movimiento
enum EstadoMovimiento {
  ESTADO_STOP = 0,
  ESTADO_RIGHT = 1,
  ESTADO_LEFT = 2,
  ESTADO_FORWARD = 3
};

EstadoMovimiento estadoActual = ESTADO_STOP;
uint8_t last_dir = 255;

void setup() {
  Serial.begin(115200);
  pinMode(DIR_IN1, INPUT);
  pinMode(DIR_IN2, INPUT);

  iniciarRecoleccion();
  
  estadoActual = ESTADO_STOP;
}

void loop() {
  procesarDireccion();
  ejecutarEstado();
  
}

void procesarDireccion() {
  uint8_t dir = (digitalRead(DIR_IN1) << 1) | digitalRead(DIR_IN2);

  if (dir != last_dir) {
    Serial.print("Dirección recibida: ");
    Serial.println(dir);

    switch (dir) {
      case 3:
        estadoActual = ESTADO_FORWARD;
        break;
      case 2:
        estadoActual = ESTADO_LEFT;
        break;
      case 1:
        estadoActual = ESTADO_RIGHT;
        break;
      case 0:
        estadoActual = ESTADO_STOP;
        break;
    }

    last_dir = dir;
  }
}

void ejecutarEstado() {
  switch (estadoActual) {
    case ESTADO_FORWARD:
     
      activarCepilloCentral();
      activarCepilloIzquierdo();
      activarCepilloDerecho();
      controlarSistemaAspersion();
      break;

    case ESTADO_LEFT:
      
      activarCepilloCentral();
      activarCepilloIzquierdo();
      activarCepilloDerecho();
      controlarSistemaAspersion();
      break;

    case ESTADO_RIGHT:
     
      activarCepilloCentral();
      activarCepilloIzquierdo();
      activarCepilloDerecho();
      controlarSistemaAspersion();
      break;

    case ESTADO_STOP:

      desactivarCepilloCentral();
      desactivarCepilloIzquierdo();
      desactivarCepilloDerecho();
      desactivarMotorSuccion();
      desactivarBombaAgua();
      break;
  }
}
