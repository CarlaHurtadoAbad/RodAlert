#include <SoftwareSerial.h>

// --- Configuración Bluetooth (HC-05) ---
SoftwareSerial BT(10, 11);  // RX, TX

// --- Pines de los sensores y buzzer ---
const int trigPin1 = 9;
const int echoPin1 = 8;
const int trigPin2 = 13;
const int echoPin2 = 12;
const int buzzerPin = 7;
const int botonPin = 6;  // botón para iniciar/finalizar sesión

// --- Variables de control ---
unsigned long startTime = 0;   // tiempo de inicio de la sesión
unsigned long endTime = 0;     // tiempo final
bool sesionActiva = false;     // indica si la sesión está corriendo
bool botonPresionado = false;  // evita múltiples lecturas del botón

// --- Variables de choques ---
int totalChoques = 0;
unsigned long tiemposChoques[50];
int indiceChoques = 0;
bool enChoque = false;

// --- Función para leer un sensor y devolver distancia (cm) ---
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 60000);
  if (duration == 0) return -1;
  return duration * 0.034 / 2.0;
}

void setup() {
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(botonPin, INPUT_PULLUP); // botón con resistencia interna

  Serial.begin(9600);
  BT.begin(9600);

  Serial.println("Presiona el botón para iniciar la sesión...");
  BT.println("Presiona el botón para iniciar la sesión...");
}

void loop() {
  // --- Control del botón ---
  if (digitalRead(botonPin) == LOW && !botonPresionado) {
    botonPresionado = true;
    delay(200); // anti-rebote simple

    if (!sesionActiva) {
      // Iniciar sesión
      sesionActiva = true;
      totalChoques = 0;
      indiceChoques = 0;
      enChoque = false;
      startTime = millis();

      Serial.println("---- SESIÓN INICIADA ----");
      BT.println("---- SESIÓN INICIADA ----");
    } else {
      // Finalizar sesión
      sesionActiva = false;
      endTime = millis();
      unsigned long duracion = (endTime - startTime) / 1000;

      Serial.println("---- SESIÓN FINALIZADA ----");
      Serial.print("Duración total: ");
      Serial.print(duracion);
      Serial.println(" segundos");
      Serial.print("Total de choques: ");
      Serial.println(totalChoques);

      BT.println("---- SESIÓN FINALIZADA ----");
      BT.print("Duración total: ");
      BT.print(duracion);
      BT.println(" s");
      BT.print("Total de choques: ");
      BT.println(totalChoques);
      BT.println("--------------------------");
    }
  } 
  else if (digitalRead(botonPin) == HIGH && botonPresionado) {
    botonPresionado = false; // espera a que se suelte el botón
  }

  // --- Si la sesión no está activa, no hace nada ---
  if (!sesionActiva) return;

  // --- Lectura de sensores ---
  float distance1 = readUltrasonic(trigPin1, echoPin1);
  delay(50);
  float distance2 = readUltrasonic(trigPin2, echoPin2);

  unsigned long tiempoActual = (millis() - startTime) / 1000;

  // --- Detección de choques ---
  bool choqueAhora = ((distance1 > 0 && distance1 <= 4) || (distance2 > 0 && distance2 <= 4));

  if (choqueAhora && !enChoque) {
    enChoque = true;
    totalChoques++;

    if (indiceChoques < 50) {
      tiemposChoques[indiceChoques++] = tiempoActual;
    }

    BT.println("⚠️ ¡Choque detectado!");
    BT.print("Número: ");
    BT.println(totalChoques);
    BT.print("Tiempo: ");
    BT.print(tiempoActual);
    BT.println(" s");
    BT.println("--------------------------");

    Serial.print("Choque #");
    Serial.print(totalChoques);
    Serial.print(" en ");
    Serial.print(tiempoActual);
    Serial.println(" s");
  }

  if (!choqueAhora && enChoque) {
    enChoque = false;
  }

  // --- Buzzer ---
  if ((distance1 > 0 && distance1 <= 10) || (distance2 > 0 && distance2 <= 10)) {
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  // --- Mostrar datos en tiempo real (formato limpio) ---
  BT.println("--------------------------");
  BT.print("Sensor 1: ");
  BT.print(distance1, 1);
  BT.println(" cm");

  BT.print("Sensor 2: ");
  BT.print(distance2, 1);
  BT.println(" cm");

  BT.print("Choques: ");
  BT.println(totalChoques);

  BT.print("Tiempo: ");
  BT.print(tiempoActual);
  BT.println(" s");
  BT.println("--------------------------");
  BT.println(); // línea en blanco

  // --- Serial Monitor (para PC) ---
  Serial.print("S1: ");
  Serial.print(distance1, 1);
  Serial.print(" cm | S2: ");
  Serial.print(distance2, 1);
  Serial.print(" cm | Choques: ");
  Serial.print(totalChoques);
  Serial.print(" | Tiempo: ");
  Serial.print(tiempoActual);
  Serial.println(" s");

  delay(500);
}
