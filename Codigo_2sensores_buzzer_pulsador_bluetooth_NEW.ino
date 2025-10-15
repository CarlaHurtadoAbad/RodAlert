#include <SoftwareSerial.h>

// --- Bluetooth (HC-05) ---
SoftwareSerial BT(10, 11);  // RX, TX

// --- Pines de sensores y buzzer ---
const int trigPin1  = 9;
const int echoPin1  = 8;
const int trigPin2  = 13;
const int echoPin2  = 12;
const int buzzerPin = 7;
const int botonPin  = 6;  // botón iniciar/finalizar sesión

// --- Estado de sesión ---
unsigned long startTime = 0;
unsigned long endTime   = 0;
bool sesionActiva       = false;
bool botonPresionado    = false;

// --- Choques ---
int totalChoques = 0;
unsigned long tiemposChoques[50];  // s desde inicio
int indiceChoques = 0;
bool enChoque = false;

// --- Datos de cabecera ---
String nombrePaciente = "";
String tipoCircuito   = "";

// --- Variables para entrada no bloqueante ---
String inputBuf = "";
unsigned long lastPromptMs = 0;
bool pidiendoNombre   = true;
bool pidiendoCircuito = false;

// --- Utilidad para imprimir en Serial y BT ---
void sendBoth(const String &s) {
  Serial.println(s);
  BT.println(s);
}

// --- Leer entrada no bloqueante de Serial y Bluetooth ---
void pumpInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (pidiendoNombre && inputBuf.length() > 0) {
        nombrePaciente = inputBuf;
        inputBuf = "";
        pidiendoNombre = false;
        pidiendoCircuito = true;
        sendBoth("Nombre OK: " + nombrePaciente);
        sendBoth("Ahora escribe el CIRCUITO y pulsa Enter...");
      } else if (pidiendoCircuito && inputBuf.length() > 0) {
        tipoCircuito = inputBuf;
        inputBuf = "";
        pidiendoCircuito = false;
        sendBoth("Circuito OK: " + tipoCircuito);
        sendBoth("----------------------------------------");
        sendBoth("Pulsa el BOTON (pin 6) para INICIAR la sesion.");
        sendBoth("========================================");
      } else {
        inputBuf = "";
      }
      return;
    } else {
      inputBuf += c;
    }
  }

  while (BT.available()) {
    char c = BT.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (pidiendoNombre && inputBuf.length() > 0) {
        nombrePaciente = inputBuf;
        inputBuf = "";
        pidiendoNombre = false;
        pidiendoCircuito = true;
        sendBoth("Nombre OK: " + nombrePaciente);
        sendBoth("Ahora escribe el CIRCUITO y pulsa Enter...");
      } else if (pidiendoCircuito && inputBuf.length() > 0) {
        tipoCircuito = inputBuf;
        inputBuf = "";
        pidiendoCircuito = false;
        sendBoth("Circuito OK: " + tipoCircuito);
        sendBoth("----------------------------------------");
        sendBoth("Pulsa el BOTON (pin 6) para INICIAR la sesion.");
        sendBoth("========================================");
      } else {
        inputBuf = "";
      }
      return;
    } else {
      inputBuf += c;
    }
  }
}

// --- Lectura ultrasónico (cm) ---
