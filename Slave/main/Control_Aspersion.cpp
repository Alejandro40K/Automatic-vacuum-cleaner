///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  PROYECTO MODULAR ASPIRADORA AUTOMATA USO RUDO
//
// Editores:           Alejando Orozco Romo
// Archivo:            Control_Aspersion.cpp
// Fecha de edicion:   25/04/2025
// Descripcion:        

// Notas:  QUITA LOS PUTOS DELAYS -----  TE PARAN TODO EL PUTO SISTEMA, SOLO HAS FUNCIONES QUE RETORNEN LOS DATOS DE LOS SENSORES 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//******************************************************************BIBLIOTECAS**********************************************************************//
#include "Control_Aspersion.h"
#include "Control_Recoleccion.h"

//*********************************************************VARIABLES PARA EL CONTROL DE ASPERSION****************************************************//
float porcentajeLleno = 0;     
unsigned long echoStartTime = 0;
unsigned long echoEndTime   = 0;
bool waitingEcho = false;

unsigned long lastActivation = 0;
const unsigned long bombaEncendidaTiempo = 3000; 
bool bombaEncendida = false;

// Constantes
const float maxDistance = 10.0; 
const float minDistance = 0.0;  
const float speed = 0.01715;    


//*********************************************************FUNCION DE INICIALIZACION***************************************************//

// FUNCIONES DE INICIO Y PRUEBA 
void iniciarAspersion(){
  pinMode(TRIGGER_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT);
}

//*********************************************************FUNCION PARA MEDIR NIVEL DEL TANQUE***************************************************//
void medirNivelTanque() {
    if (!waitingEcho) {
        digitalWrite(TRIGGER_PIN, HIGH);
        delayMicroseconds(10);  // 10us de pulso de trigger
        digitalWrite(TRIGGER_PIN, LOW);
        echoStartTime = micros();
        waitingEcho = true;
    }
}

//*********************************************************FUNCION ACTUALIZA EL NIVEL DEL TANQUE***************************************************//
void actualizarNivelTanque(){
    if (waitingEcho) {
        if (digitalRead(ECHO_PIN)) {
            // Esperando que vuelva a LOW para calcular duración
        } else {
            echoEndTime = micros();
            long duration = echoEndTime - echoStartTime; // tiempo en us
            float distance = duration * speed;           // distancia en cm

            // Mapear a porcentaje de llenado
            porcentajeLleno = (maxDistance - distance) * 100.0 / (maxDistance - minDistance);
            if (porcentajeLleno < 0) porcentajeLleno = 0;
            if (porcentajeLleno > 100) porcentajeLleno = 100;

            waitingEcho = false;
        }
    }
}

//*********************************************************FUNCION CONTROL ASPERSION***************************************************//
void controlarSistemaAspersion(){
    // Activar bomba si hay suficiente agua
    if (porcentajeLleno >= 10) {
        if (!bombaEncendida) {
            activarBombaAgua();
            lastActivation = millis();
            bombaEncendida = true;
        }
    }

    // Apagar bomba después de tiempo definido
    if (bombaEncendida && (millis() - lastActivation >= bombaEncendidaTiempo)) {
        desactivarBombaAgua();
        bombaEncendida = false;
    }

    // Apagar bomba si el nivel es bajo
    if (porcentajeLleno < 10) {
        desactivarBombaAgua();
        bombaEncendida = false;
    }
}

//*********************************************************FUNCION INICIO ASPERSION***************************************************//
void actualizarAspersion(){
    medirNivelTanque();
    actualizarNivelTanque();
    controlarSistemaAspersion();
}
