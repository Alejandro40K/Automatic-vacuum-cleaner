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
#include <WebSocketsClient.h>

// Red Wi-Fi a conectar
const char* ssid = "MEGACABLE-2.4G-B2A6";
const char* password = "tc3ZtzGG47";

// IP y puerto del servidor WebSocket (tu PC)
const char* server_ip = "192.168.100.10";  // Ajusta esta IP a la de tu computadora
const int server_port = 8084;

// Pines de salida para control binario
#define DIR_OUT1 2
#define DIR_OUT2 15

WebSocketsClient webSocket;

// Evento de recepción WebSocket
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String comando = String((char*)payload);
    Serial.print("Comando recibido: ");
    Serial.println(comando);

    if (comando == "forward") {
      digitalWrite(DIR_OUT1, HIGH);
      digitalWrite(DIR_OUT2, HIGH);
    } else if (comando == "left") {
      digitalWrite(DIR_OUT1, HIGH);
      digitalWrite(DIR_OUT2, LOW);
    } else if (comando == "right") {
      digitalWrite(DIR_OUT1, LOW);
      digitalWrite(DIR_OUT2, HIGH);
    } else if (comando == "stop") {
      digitalWrite(DIR_OUT1, LOW);
      digitalWrite(DIR_OUT2, LOW);
    }

    webSocket.sendTXT("ACK desde ESP32");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(DIR_OUT1, OUTPUT);
  pinMode(DIR_OUT2, OUTPUT);
  digitalWrite(DIR_OUT1, LOW);
  digitalWrite(DIR_OUT2, LOW);

  Serial.println("Iniciando conexión WiFi...");
  WiFi.begin(ssid, password);

  // Espera a conexión
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado");
    Serial.println(WiFi.localIP());
    webSocket.begin(server_ip, server_port, "/");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
  } else {
    Serial.println("\nNo se pudo conectar a WiFi");
  }
}

void loop() {
  webSocket.loop();
}

























/// DE PRUEBA 
/*#include <WiFi.h>
#include <WebSocketsClient.h>

const char* ssid = "MEGACABLE-2.4G-B2A6";   // SSID de wifi
const char* password = "tc3ZtzGG47";        // Contraseña wifi

const char* server_ip = "192.168.100.10";  // IP de la PC con Python
const int server_port = 8084;

WebSocketsClient webSocket;
unsigned long lastSend = 0;

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    Serial.print("Mensaje recibido de Python: ");
    Serial.println((char*)payload);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }

  Serial.println("\n Conectado a WiFi");
  Serial.println(WiFi.localIP());

  webSocket.begin(server_ip, server_port, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  webSocket.loop();

  if (millis() - lastSend > 2000) {
    webSocket.sendTXT("Hola desde ESP32");
    Serial.println("Mensaje enviado a Python");
    lastSend = millis();
  }
}

*/








