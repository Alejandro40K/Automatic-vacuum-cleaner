///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  PROYECTO MODULAR ASPIRADORA AUTOMATA USO RUDO
//
// Editores:           Alejando Orozco Romo
// Archivo:            master / main.ino
// Fecha de creación:   24/04/2025
// Fecha de edición:   09/05/2025
// Descripcion:         
// Notas: 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <WiFi.h>

//*******************************************************CONFIGURACION WIFI********************************************************//
const char* ssid = "MEGACABLE-2.4G-B2A6";
const char* password = "tc3ZtzGG47";

// IP del ESP32 (local) y puerto TCP
const int serverPort = 8084;

//*******************************************************CONFIGURACION UART********************************************************//
#define UART_TX 2   
#define UART_RX 15  
#define UART_BAUD 115200

WiFiServer server(serverPort);
WiFiClient client;

//*******************************************************FUNCION SETUP********************************************************//
void setup() {
  Serial.begin(115200);  // Monitor en PC
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX); // UART hacia esclavo

  // Conexión Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Conectando WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado: " + WiFi.localIP().toString());

  // Iniciar servidor TCP
  server.begin();
  Serial.println("Servidor TCP iniciado en puerto " + String(serverPort));
}

//*******************************************************FUNCION LOOP********************************************************//
void loop() {
  // Aceptar cliente
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("Cliente conectado: " + client.remoteIP().toString());
    }
  }

  // Leer datos del cliente y reenviar por UART
  if (client && client.connected() && client.available()) {
    String data = client.readStringUntil('\n'); // leer hasta salto de línea
    data.trim();
    if (data.length() > 0) {
      Serial.println("Velocidades recibidas: " + data);
      Serial2.println(data); // enviar al esclavo
    }
  }

  // Leer datos de UART y enviar al cliente TCP
  if (Serial2.available()) {
    String dataFromSlave = Serial2.readStringUntil('\n');
    dataFromSlave.trim();
    if (client && client.connected() && dataFromSlave.length() > 0) {
      client.println(dataFromSlave); // enviar datos medidos al cliente
    }
  }
}