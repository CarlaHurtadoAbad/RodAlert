/*  PRUEBA 3 – Ruido Temporal
    - Mide a 0.2 m, 0.5 m, 1.0 m y 1.5 m (20, 50, 100, 150 cm)
    - Si contestas SI, mide 60 s en estático y emite los datos por Bluetooth en formato CSV
    Formato de salida:
    #DIST_CM,T_MS,S1_CM,S2_CM,MIN_CM
    20,50,20.4,20.5,20.4
*/

#include <Arduino.h>

// ---------- Pines ----------
#define TRIG1 12
#define ECHO1 11
#define TRIG2 9
#define ECHO2 8

// ---------- Bluetooth ----------
#define BT Serial1
const unsigned long BT_BAUD = 9600;

// ---------- Config ----------
const unsigned long WINDOW_MS        = 60000UL; // 60 s
const unsigned long SAMPLE_PERIOD_MS = 50UL;    // 20 Hz
const int DISTANCES_CM[] = {20, 50, 100, 150};
const int NUM_STEPS = sizeof(DISTANCES_CM) / sizeof(DISTANCES_CM[0]);

// ---------- Estado ----------
enum State { WAIT_CONFIRM, SAMPLING, DONE };
State state = WAIT_CONFIRM;
int stepIndex = 0;

unsigned long startMs = 0;
unsigned long lastSampleMs = 0;
unsigned long sentSamples = 0;
unsigned long validSamples = 0;

// ---------- RX BT ----------
String btLine = "";

// ---------- Funciones ----------
float readUltrasonicCM(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long dur = pulseIn(echo, HIGH, 30000UL);
  if (dur == 0) return -1.0f;
  return (dur * 0.0343f) / 2.0f;
}

void sendBoth(const String& s) { BT.println(s); Serial.println(s); }

void promptCurrentDistance() {
  if (stepIndex < NUM_STEPS)
    sendBoth("¿ESTÁS A " + String(DISTANCES_CM[stepIndex]) + " cm? (SI/NO)");
}

// ---------- Setup ----------
void setup() {
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  Serial.begin(115200);
  BT.begin(BT_BAUD);
  delay(200);

  sendBoth("PRUEBA3 - Ruido Temporal");
  promptCurrentDistance();
}

// ---------- Loop ----------
void loop() {
  // --- RX por Bluetooth ---
  while (BT.available()) {
    char c = (char)BT.read();
    if (c == '\r') continue;
    if (c == '\n') {
      btLine.trim();
      if (btLine.length() > 0) {
        String cmdUp = btLine;
        cmdUp.trim();
        cmdUp.toUpperCase();
        btLine = "";

        if (state == WAIT_CONFIRM && stepIndex < NUM_STEPS) {
          if (cmdUp == "SI") {
            startMs = millis();
            lastSampleMs = 0;
            sentSamples = 0;
            validSamples = 0;
            state = SAMPLING;
            sendBoth("OK, MIDE 60 s A " + String(DISTANCES_CM[stepIndex]) + " cm...");
            sendBoth("#DIST_CM,T_MS,S1_CM,S2_CM,MIN_CM");
          } else {
            promptCurrentDistance();
          }
        }
      } else {
        btLine = "";
      }
    } else {
      btLine += c;
    }
  }

  // --- Muestreo ---
  if (state == SAMPLING) {
    unsigned long now = millis();

    if (lastSampleMs == 0 || (now - lastSampleMs) >= SAMPLE_PERIOD_MS) {
      lastSampleMs = now;

      float d1 = readUltrasonicCM(TRIG1, ECHO1);
      delay(30);
      float d2 = readUltrasonicCM(TRIG2, ECHO2);

      float minD = -1.0f;
      if (d1 > 0 && d2 > 0) minD = (d1 < d2) ? d1 : d2;
      else if (d1 > 0)      minD = d1;
      else if (d2 > 0)      minD = d2;

      // ---- CSV ----
      BT.print(DISTANCES_CM[stepIndex]); BT.print(",");
      BT.print(now - startMs); BT.print(",");
      (d1 < 0) ? BT.print("NA") : BT.print(d1, 1); BT.print(",");
      (d2 < 0) ? BT.print("NA") : BT.print(d2, 1); BT.print(",");
      (minD < 0) ? BT.println("NA") : BT.println(minD, 1);

      sentSamples++;
      if (minD >= 0.0f) validSamples++;
    }

    // --- Fin de los 60 s ---
    if ((millis() - startMs) >= WINDOW_MS) {
      sendBoth("HECHO 60 s A " + String(DISTANCES_CM[stepIndex]) + " cm. " +
               String("Muestras enviadas: ") + String(sentSamples) +
               String(", válidas: ") + String(validSamples));
      stepIndex++;
      if (stepIndex >= NUM_STEPS) {
        state = DONE;
        sendBoth("PROCESO COMPLETADO EN 20, 50, 100 y 150 cm ✅");
      } else {
        state = WAIT_CONFIRM;
        promptCurrentDistance();
      }
    }
  }
}
