/* ==============================================================
/* RodAlert MEGA 
   
   Final code with Ultrasonic Sensors monitoring:
   - 4 Ultrasonic Sensors
   - 1 Module Bluetooth
   - 1 Buzzer
   - 2 Buttons
   - 1 Motor ERM
   ============================================================== */

#include <Arduino.h>
#include <math.h>

// SERIAL / BT
#define BT Serial
const unsigned long BT_BAUD = 9600;

// Pines
#define TRIG1 13
#define ECHO1 12
#define TRIG2 11
#define ECHO2 10

// S3 cabeza (giro)
#define TRIG3 25
#define ECHO3 23

// S4 lateral obstáculos
#define TRIG4 22
#define ECHO4 24

#define BUZZER_PIN    7
#define BUTTON_BUZZER 8
#define BUTTON_MOTOR  9

// L298N canal A (Motor)
const int L298_ENA = 6;
const int L298_IN1 = 5;
const int L298_IN2 = 4;

const int chosenPWM = 255;

// Anti-parpadeo BUZZER
const unsigned long ON_DELAY_MS_BUZZER  = 30;
const unsigned long OFF_DELAY_MS_BUZZER = 450;
unsigned long buzzerCondSinceMs = 0;
unsigned long buzzerStopPendMs  = 0;

// Ultrasonidos 
const unsigned long ULTRASONIC_TIMEOUT_US = 30000;
const unsigned int  INTERSHOT_US          = 1500;
const float         TOL_CM                = 3.0f;

const bool  USE_EMA_DIST = false;
const float EMA_ALPHA    = 0.7f;
float minD_filt = -1.0f;

// Parámetros (BASE y ACTIVOS)
float BEEP_START_CM_BASE = 50.0f;  // frontal empieza pitido (discontinuo)
float BEEP_CONT_CM_BASE  = 30.0f;  // frontal continuo <= esto

float S4_BEEP_CM_BASE    = 15.0f;  // lateral pitido (continuo)
float VIBR_ON_CM_BASE    = 22.0f;  // motor frontal umbral
float S4_VIBR_CM_BASE    = 6.0f;   // motor lateral umbral
float CRASH_CM_BASE      = 6.0f;   // choque

float BEEP_START_CM = 50.0f;
float BEEP_CONT_CM  = 30.0f;
float S4_BEEP_CM    = 15.0f;
float VIBR_ON_CM    = 22.0f;
float S4_VIBR_CM    = 6.0f;
float CRASH_CM      = 6.0f;

// Beeps discontinuos (solo frontal)
const unsigned long BEEP_INTERVAL_NEAR_MS = 120;
const unsigned long BEEP_INTERVAL_FAR_MS  = 650;

// S3 (cabeza) calibración y eventos 
float s3BaseDist     = 0.0f;
float s3TurnDist     = 0.0f;
float s3DeltaThresh  = 0.0f;
bool  s3Calibrated   = false;

const float S3_TURN_FACTOR = 0.6f;
const float S3_HYST_FACTOR = 0.7f;

float s3CurrentApproach = 0.0f;

struct S3TurnEvent {
  unsigned long t_start_ms;
  unsigned long t_end_ms;
  float min_d3;
  float max_approach_cm;
};
const int MAX_S3_TURN_EVENTS = 25;
S3TurnEvent s3TurnEvents[MAX_S3_TURN_EVENTS];
int  s3TurnCount = 0;
bool s3TurnOpen  = false;
unsigned long s3TurnStartMs = 0;
float s3TurnMinD3       = 9999.0f;
float s3TurnMaxApproach = 0.0f;

// “Evento” para DATA tipo choque
enum S3Evt { S3_EVT_NONE=0, S3_EVT_START=1, S3_EVT_END=2 };
volatile S3Evt s3EvtPulse = S3_EVT_NONE;

// Sesión / métricas
bool sessionActive = false;
unsigned long sessionStartMs = 0;

// Choques (SOLO S1/S2/S4)
const int MAX_CHOQUES = 30;
int totalChoques = 0;
bool enChoque = false;

unsigned long tiemposChoques[MAX_CHOQUES];   int idxChoques   = 0;
unsigned long tiemposChoquesS1[MAX_CHOQUES]; int idxChoquesS1 = 0;
unsigned long tiemposChoquesS2[MAX_CHOQUES]; int idxChoquesS2 = 0;
unsigned long tiemposChoquesS4[MAX_CHOQUES]; int idxChoquesS4 = 0;

// Eventos buzzer/motor
const int MAX_EVENTOS_ACT = 80;
unsigned long tiemposMotorOn[MAX_EVENTOS_ACT];  int idxMotorOn  = 0;
unsigned long tiemposBuzzerOn[MAX_EVENTOS_ACT]; int idxBuzzerOn = 0;

bool buzzerEnableUser = false;
bool motorEnableUser  = false;
bool buzzerIsOn       = false;

// Seguridad
float minSessionDist = -1.0f;
int   falseNegCount  = 0;
int   falsePosCount  = 0;
float sumDelayAvisoChoque = 0.0f;
int   nDelayAvisoChoque   = 0;

// Motor pulsos
const unsigned long MOTOR_PULSE_MS = 1000;
const unsigned long PULSE_REST_MS  = 3000;
bool pulseActive = false;
unsigned long pulseStartMs = 0;
unsigned long nextAllowedPulseMs = 0;

// DATA streaming
unsigned long lastDataTxMs = 0;
const unsigned long DATA_TX_INTERVAL_MS = 250;

// FSM limpia
enum CommState {
  ASK_PATIENT_ID,
  ASK_CONDITION,
  ASK_CHANGE_PARAMS,
  SET_BEEP_START,
  SET_BEEP_CONT,
  SET_S4_BEEP,
  SET_VIBR_FRONT,
  SET_S4_VIBR,
  SET_CRASH,
  CAL_S3_BASE,
  CAL_S3_TURN,
  WAIT_START_OK,
  SESSION_RUNNING,
  ASK_NEW_SESSION
};
CommState commState = ASK_PATIENT_ID;

// Para "CAL en cualquier momento"
CommState resumeStateAfterCal = ASK_PATIENT_ID;
bool resumeToSession = false;

String btLine = "";
String patientID = "";
String conditionSelection = "";

// Recordatorio
unsigned long lastPromptMs = 0;
const unsigned long PROMPT_PERIOD_MS = 2000;

// Utils
inline void sendMsg(const String& s){ BT.println(s); }

unsigned long secsSinceStart(){
  if(!sessionActive) return 0;
  return (millis() - sessionStartMs) / 1000UL;
}

static float pulseToCm(unsigned long us){ return us / 58.0f; }

float readOnceCM(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long dur = pulseIn(echo, HIGH, ULTRASONIC_TIMEOUT_US);
  if (dur == 0) return -1.0f;
  return pulseToCm(dur);
}

float median3(float a, float b, float c) {
  if (a > b) { float t=a; a=b; b=t; }
  if (b > c) { float t=b; b=c; c=t; }
  if (a > b) { float t=a; a=b; b=t; }
  return b;
}

float readUltrasonicSmart(uint8_t trig, uint8_t echo) {
  if (commState != SESSION_RUNNING || !sessionActive) return -1.0f;

  float r1 = readOnceCM(trig, echo);
  delayMicroseconds(INTERSHOT_US);
  float r2 = readOnceCM(trig, echo);

  if (r1 < 0 || r2 < 0 || fabs(r1 - r2) > TOL_CM) {
    delayMicroseconds(INTERSHOT_US);
    float r3 = readOnceCM(trig, echo);

    float v[3] = {r1, r2, r3};
    float vals[3]; int nValid = 0;
    for (int i=0;i<3;i++) if (v[i] >= 0) vals[nValid++] = v[i];

    if (nValid == 0) return -1.0f;
    if (nValid == 1) return vals[0];
    if (nValid == 2) return (fabs(vals[0]-vals[1])<=TOL_CM) ? (vals[0]+vals[1])*0.5f : min(vals[0], vals[1]);
    return median3(vals[0], vals[1], vals[2]);
  }
  return (r1 + r2) * 0.5f;
}

float calibrateS3For(unsigned long msWindow) {
  unsigned long t0 = millis();
  float suma = 0.0f;
  int n = 0;
  while (millis() - t0 < msWindow) {
    float d = readOnceCM(TRIG3, ECHO3);
    if (d > 0) { suma += d; n++; }
    delay(40);
  }
  if (n == 0) return -1.0f;
  return suma / n;
}

// Actuadores
void motorPWM(int pwm){ pwm=constrain(pwm,0,255); analogWrite(L298_ENA,pwm); }
bool motorRunning=false;

void motorOn(){
  if(!motorRunning){
    motorRunning=true;
    digitalWrite(L298_IN1,HIGH);
    digitalWrite(L298_IN2,LOW);
    motorPWM(chosenPWM);
    if(sessionActive && idxMotorOn < MAX_EVENTOS_ACT) tiemposMotorOn[idxMotorOn++] = secsSinceStart();
  }
}
void motorOff(){
  if(motorRunning){
    motorRunning=false;
    analogWrite(L298_ENA,0);
  }
}

// Buzzer activo
void setBuzzerEnabled(bool en){
  if(en && !buzzerIsOn){
    digitalWrite(BUZZER_PIN, LOW);
    buzzerIsOn = true;
    if(sessionActive && idxBuzzerOn < MAX_EVENTOS_ACT) tiemposBuzzerOn[idxBuzzerOn++] = secsSinceStart();
  } else if(!en && buzzerIsOn){
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerIsOn = false;
  }
}

// Parámetros 
void restoreBaseParams(){
  BEEP_START_CM = BEEP_START_CM_BASE;
  BEEP_CONT_CM  = BEEP_CONT_CM_BASE;
  S4_BEEP_CM    = S4_BEEP_CM_BASE;
  VIBR_ON_CM    = VIBR_ON_CM_BASE;
  S4_VIBR_CM    = S4_VIBR_CM_BASE;
  CRASH_CM      = CRASH_CM_BASE;
}

static bool parseFloatFromCmd(const String& raw, float &out){
  String s = raw;
  s.trim();
  if(s.length()==0) return false;

  bool hasDigit = false;
  for (unsigned int i=0; i<s.length(); i++){
    char ch = s[i];
    if ((ch >= '0' && ch <= '9')) { hasDigit = true; break; }
  }
  if(!hasDigit) return false;

  out = s.toFloat();
  if(!isfinite(out)) return false;
  return true;
}

void printCurrentParams(){
  sendMsg("PARAMETROS ACTIVOS:");
  sendMsg("  Buzzer frontal: start=" + String(BEEP_START_CM,1) + " cm, cont=" + String(BEEP_CONT_CM,1) + " cm");
  sendMsg("  Buzzer S4: " + String(S4_BEEP_CM,1) + " cm");
  sendMsg("  Motor frontal: " + String(VIBR_ON_CM,1) + " cm");
  sendMsg("  Motor S4: " + String(S4_VIBR_CM,1) + " cm");
  sendMsg("  Crash: " + String(CRASH_CM,1) + " cm");
}

// Reset sesión
void resetSessionMetrics(){
  totalChoques=0; enChoque=false;
  idxChoques=0; idxChoquesS1=0; idxChoquesS2=0; idxChoquesS4=0;

  idxMotorOn=0; idxBuzzerOn=0;

  minSessionDist=-1.0f;
  falseNegCount=0; falsePosCount=0; sumDelayAvisoChoque=0.0f; nDelayAvisoChoque=0;

  s3TurnCount=0; s3TurnOpen=false; s3TurnStartMs=0; s3TurnMinD3=9999.0f; s3TurnMaxApproach=0.0f; s3CurrentApproach=0.0f;
  s3EvtPulse = S3_EVT_NONE;

  minD_filt=-1.0f;

  pulseActive=false;
  nextAllowedPulseMs = millis();
  motorOff();
  setBuzzerEnabled(false);
  buzzerCondSinceMs=0;
  buzzerStopPendMs=0;
}

// CAL en cualquier momento
void startRecalS3(){
  resumeStateAfterCal = commState;
  resumeToSession = (commState == SESSION_RUNNING && sessionActive);

  motorOff();
  setBuzzerEnabled(false);

  s3Calibrated=false;
  sendMsg("=== CALIBRACION S3 (RECAL) ===");
  sendMsg("1) Cabeza recta. Escriba OK para medir 1 s.");
  commState = CAL_S3_BASE;
}

void computeSafetyMetrics(){
  falseNegCount = 0;
  falsePosCount = 0;
  sumDelayAvisoChoque = 0.0f;
  nDelayAvisoChoque   = 0;

  bool hayAsistencia = (conditionSelection != "1");
  if(!hayAsistencia) return;

  // FN + delay aviso->choque (3s)
  for(int i=0;i<idxChoques;i++){
    unsigned long tC = tiemposChoques[i];
    bool  hadWarning = false;
    float lastWarn   = -1.0f;

    for(int j=0;j<idxBuzzerOn;j++){
      float tB = (float)tiemposBuzzerOn[j];
      if(tB >= (float)tC - 3.0f && tB <= (float)tC){
        hadWarning = true;
        if(tB > lastWarn) lastWarn = tB;
      }
    }
    for(int j=0;j<idxMotorOn;j++){
      float tM = (float)tiemposMotorOn[j];
      if(tM >= (float)tC - 3.0f && tM <= (float)tC){
        hadWarning = true;
        if(tM > lastWarn) lastWarn = tM;
      }
    }

    if(!hadWarning) falseNegCount++;
    else if(lastWarn >= 0.0f){
      sumDelayAvisoChoque += ((float)tC - lastWarn);
      nDelayAvisoChoque++;
    }
  }

  // FP aviso sin choque en 5s
  for(int j=0;j<idxBuzzerOn;j++){
    float tB = (float)tiemposBuzzerOn[j];
    bool choqueAfter=false;
    for(int i=0;i<idxChoques;i++){
      float tC=(float)tiemposChoques[i];
      if(tC >= tB && tC <= tB+5.0f){ choqueAfter=true; break; }
    }
    if(!choqueAfter) falsePosCount++;
  }
  for(int j=0;j<idxMotorOn;j++){
    float tM = (float)tiemposMotorOn[j];
    bool choqueAfter=false;
    for(int i=0;i<idxChoques;i++){
      float tC=(float)tiemposChoques[i];
      if(tC >= tM && tC <= tM+5.0f){ choqueAfter=true; break; }
    }
    if(!choqueAfter) falsePosCount++;
  }
}

void printSessionSummary(unsigned long durS){
  sendMsg("---- RESUMEN DE LA SESION ----");
  sendMsg("Duracion: " + String(durS) + " s");

  if(minSessionDist>0) sendMsg("Distancia minima MINf: " + String(minSessionDist,1) + " cm");
  else sendMsg("Distancia minima MINf: NA");

  // Choques globales
  sendMsg("Choques totales: " + String(totalChoques));
  for(int i=0;i<idxChoques;i++){
    sendMsg("  Choque_global #" + String(i+1) + " en t=" + String(tiemposChoques[i]) + " s");
  }

  // Choques por sensor (SIN S3)
  sendMsg("Choques por sensor:");
  sendMsg("  S1 frontal: " + String(idxChoquesS1));
  for(int i=0;i<idxChoquesS1;i++){
    sendMsg("    Choque_S1 #" + String(i+1) + " en t=" + String(tiemposChoquesS1[i]) + " s");
  }
  sendMsg("  S2 frontal: " + String(idxChoquesS2));
  for(int i=0;i<idxChoquesS2;i++){
    sendMsg("    Choque_S2 #" + String(i+1) + " en t=" + String(tiemposChoquesS2[i]) + " s");
  }
  sendMsg("  S4 lateral: " + String(idxChoquesS4));
  for(int i=0;i<idxChoquesS4;i++){
    sendMsg("    Choque_S4 #" + String(i+1) + " en t=" + String(tiemposChoquesS4[i]) + " s");
  }

  // Buzzer
  sendMsg("Activaciones buzzer: " + String(idxBuzzerOn));
  for(int i=0;i<idxBuzzerOn;i++){
    sendMsg("  Buzzer #" + String(i+1) + " en t=" + String(tiemposBuzzerOn[i]) + " s");
  }

  // Motor
  sendMsg("Activaciones motor: " + String(idxMotorOn));
  for(int i=0;i<idxMotorOn;i++){
    sendMsg("  Motor #" + String(i+1) + " en t=" + String(tiemposMotorOn[i]) + " s");
  }

  // Giros S3
  sendMsg("Giros S3 (cabeza izq): " + String(s3TurnCount));
  for(int i=0;i<s3TurnCount;i++){
    float t_ini_s = s3TurnEvents[i].t_start_ms/1000.0f;
    float t_fin_s = s3TurnEvents[i].t_end_ms/1000.0f;
    float dur = t_fin_s - t_ini_s;

    String line = "  Giro_S3 #";
    line += String(i+1);
    line += " de t=" + String(t_ini_s,1);
    line += " s a t=" + String(t_fin_s,1);
    line += " s (dur=" + String(dur,1);
    line += " s, min_d3=" + String(s3TurnEvents[i].min_d3,1);
    line += " cm, max_approach=" + String(s3TurnEvents[i].max_approach_cm,1) + " cm)";
    sendMsg(line);
  }

  // Seguridad
  bool hayAsistencia = (conditionSelection != "1");
  if(hayAsistencia){
    computeSafetyMetrics();
    sendMsg("Choques sin aviso previo en 3 s (falsos negativos): " + String(falseNegCount));
    sendMsg("Avisos sin choque en 5 s (falsos positivos): " + String(falsePosCount));
    if(nDelayAvisoChoque>0){
      float delayMedio = sumDelayAvisoChoque/(float)nDelayAvisoChoque;
      sendMsg("Tiempo medio aviso->choque: " + String(delayMedio,2) + " s (n=" + String(nDelayAvisoChoque) + ")");
    } else {
      sendMsg("Tiempo medio aviso->choque: NA");
    }
  } else {
    sendMsg("Condicion SIN ASISTENCIA: no se calculan falsos positivos/negativos.");
  }
}

// Setup
void setup(){
  pinMode(TRIG1,OUTPUT); pinMode(ECHO1,INPUT);
  pinMode(TRIG2,OUTPUT); pinMode(ECHO2,INPUT);
  pinMode(TRIG3,OUTPUT); pinMode(ECHO3,INPUT);
  pinMode(TRIG4,OUTPUT); pinMode(ECHO4,INPUT);

  pinMode(BUZZER_PIN,OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);

  pinMode(BUTTON_BUZZER, INPUT_PULLUP);
  pinMode(BUTTON_MOTOR,  INPUT_PULLUP);

  pinMode(L298_ENA,OUTPUT); pinMode(L298_IN1,OUTPUT); pinMode(L298_IN2,OUTPUT);
  digitalWrite(L298_IN1,HIGH);
  digitalWrite(L298_IN2,LOW);
  analogWrite(L298_ENA,0);

  BT.begin(BT_BAUD);
  delay(200);

  restoreBaseParams();

  sendMsg("=== RodAlert boot (TFM-clean v3, sin IMU) ===");
  sendMsg("ID PACIENTE?");
  lastPromptMs = millis();
}

// Loop
void loop(){
  unsigned long nowMs = millis();

  // BOTONES (siempre activos)
  static int lastBz = HIGH, lastMo = HIGH;
  static unsigned long lastBzChangeMs=0, lastMoChangeMs=0;

  int rb = digitalRead(BUTTON_BUZZER);
  if(rb!=lastBz && (nowMs - lastBzChangeMs > 150)){
    lastBzChangeMs = nowMs;
    lastBz = rb;
    if(rb==LOW){
      buzzerEnableUser = !buzzerEnableUser;
      if(!buzzerEnableUser) setBuzzerEnabled(false);
      sendMsg(buzzerEnableUser ? "BUZZER ENABLED" : "BUZZER DISABLED");
    }
  }

  int rm = digitalRead(BUTTON_MOTOR);
  if(rm!=lastMo && (nowMs - lastMoChangeMs > 150)){
    lastMoChangeMs = nowMs;
    lastMo = rm;
    if(rm==LOW){
      motorEnableUser = !motorEnableUser;
      if(!motorEnableUser) motorOff();
      sendMsg(motorEnableUser ? "MOTOR ENABLED" : "MOTOR DISABLED");
    }
  }

  // SENSORES durante sesión 
  float d1=-1, d2=-1, d3=-1, d4=-1;

  if(commState == SESSION_RUNNING && sessionActive){
    d1 = readUltrasonicSmart(TRIG1,ECHO1);
    delayMicroseconds(1000);
    d2 = readUltrasonicSmart(TRIG2,ECHO2);
    delayMicroseconds(1000);
    d3 = readOnceCM(TRIG3,ECHO3);
    d4 = readOnceCM(TRIG4,ECHO4);

    // minD (SOLO S1/S2/S4)
    float minD=-1.0f;
    float arrD[3] = {d1,d2,d4};
    for(int i=0;i<3;i++){
      if(arrD[i] > 0 && (minD<0 || arrD[i] < minD)) minD = arrD[i];
    }

    if (USE_EMA_DIST) {
      if (minD >= 0) {
        if (minD_filt < 0) minD_filt = minD;
        else minD_filt = EMA_ALPHA*minD_filt + (1.0f-EMA_ALPHA)*minD;
      }
    } else minD_filt = minD;

    if(minD_filt > 0){
      if(minSessionDist < 0 || minD_filt < minSessionDist) minSessionDist = minD_filt;
    }

    // CHOQUES 
    bool choqueAhora = (minD_filt>0 && minD_filt<=CRASH_CM);
    if(choqueAhora && !enChoque){
      enChoque=true;
      totalChoques++;
      unsigned long tNow = secsSinceStart();

      if(idxChoques<MAX_CHOQUES) tiemposChoques[idxChoques++] = tNow;

      bool c1 = (d1>0 && d1<=CRASH_CM);
      bool c2 = (d2>0 && d2<=CRASH_CM);
      bool c4 = (d4>0 && d4<=CRASH_CM);

      if(c1 && idxChoquesS1<MAX_CHOQUES) tiemposChoquesS1[idxChoquesS1++] = tNow;
      if(c2 && idxChoquesS2<MAX_CHOQUES) tiemposChoquesS2[idxChoquesS2++] = tNow;
      if(c4 && idxChoquesS4<MAX_CHOQUES) tiemposChoquesS4[idxChoquesS4++] = tNow;

      sendMsg("Choque detectado");
    }
    if(!choqueAhora && enChoque) enChoque=false;

    // GIROS S3
    if(s3Calibrated){
      bool haveD3 = (d3 > 0);
      float approach = 0.0f;

      if(haveD3 && s3BaseDist > 0){
        approach = s3BaseDist - d3;
        if(approach < 0) approach = 0;
      }
      s3CurrentApproach = approach;

      bool turnedNow = (haveD3 && approach >= s3DeltaThresh);
      bool keepTurn  = (haveD3 && approach >= s3DeltaThresh * S3_HYST_FACTOR);

      if(!s3TurnOpen && turnedNow){
        s3TurnOpen = true;
        s3EvtPulse = S3_EVT_START;
        s3TurnStartMs = secsSinceStart()*1000UL;
        s3TurnMinD3 = d3;
        s3TurnMaxApproach = approach;
      } else if(s3TurnOpen){
        if(haveD3 && d3 < s3TurnMinD3) s3TurnMinD3 = d3;
        if(approach > s3TurnMaxApproach) s3TurnMaxApproach = approach;

        if(!keepTurn || !haveD3){
          if(s3TurnCount < MAX_S3_TURN_EVENTS){
            unsigned long t_end = secsSinceStart()*1000UL;
            s3TurnEvents[s3TurnCount].t_start_ms = s3TurnStartMs;
            s3TurnEvents[s3TurnCount].t_end_ms   = t_end;
            s3TurnEvents[s3TurnCount].min_d3     = s3TurnMinD3;
            s3TurnEvents[s3TurnCount].max_approach_cm = s3TurnMaxApproach;
            s3TurnCount++;
          }
          s3TurnOpen=false;
          s3EvtPulse = S3_EVT_END;
          s3CurrentApproach=0.0f;
          s3TurnMinD3=9999.0f;
          s3TurnMaxApproach=0.0f;
        }
      }
    }

    // BUZZER
    static bool beepingNow=false;
    static unsigned long lastBeepToggleMs=0;

    float buzzerDistFront = -1.0f;
    float buzzerDistLat   = -1.0f;

    if (d1 > 0 && d1 <= BEEP_START_CM) buzzerDistFront = (buzzerDistFront<0)? d1 : min(buzzerDistFront,d1);
    if (d2 > 0 && d2 <= BEEP_START_CM) buzzerDistFront = (buzzerDistFront<0)? d2 : min(buzzerDistFront,d2);
    if (d4 > 0 && d4 <= S4_BEEP_CM) buzzerDistLat = d4;

    if(buzzerEnableUser){
      if(buzzerDistLat >= 0){
        buzzerCondSinceMs = 0;
        beepingNow = false;
        setBuzzerEnabled(true);
        buzzerStopPendMs = 0;
      }
      else if(buzzerDistFront >= 0){
        if(buzzerDistFront <= BEEP_CONT_CM){
          if(buzzerCondSinceMs==0) buzzerCondSinceMs = nowMs;
          if(nowMs - buzzerCondSinceMs >= ON_DELAY_MS_BUZZER){
            setBuzzerEnabled(true);
            buzzerStopPendMs = 0;
          }
        } else {
          float span = max(1.0f, (BEEP_START_CM - BEEP_CONT_CM));
          float x = (BEEP_START_CM - buzzerDistFront) / span;
          x = constrain(x, 0.0f, 1.0f);
          unsigned long interval = (unsigned long)((1.0f - x)*BEEP_INTERVAL_FAR_MS + x*BEEP_INTERVAL_NEAR_MS);

          if(nowMs - lastBeepToggleMs >= interval){
            lastBeepToggleMs = nowMs;
            beepingNow = !beepingNow;
            setBuzzerEnabled(beepingNow);
          }
        }
      }
      else {
        buzzerCondSinceMs = 0;
        beepingNow = false;
        if(buzzerIsOn){
          if(buzzerStopPendMs==0) buzzerStopPendMs = nowMs;
          if(nowMs - buzzerStopPendMs >= OFF_DELAY_MS_BUZZER){
            setBuzzerEnabled(false);
            buzzerStopPendMs = 0;
          }
        }
      }
    } else {
      buzzerCondSinceMs = 0;
      buzzerStopPendMs  = 0;
      beepingNow=false;
      setBuzzerEnabled(false);
    }

    // MOTOR
    float motorDistFront = -1.0f;
    if (d1 > 0) motorDistFront = (motorDistFront<0)? d1 : min(motorDistFront,d1);
    if (d2 > 0) motorDistFront = (motorDistFront<0)? d2 : min(motorDistFront,d2);

    bool near = false;
    if(motorEnableUser){
      if(motorDistFront >= 0 && motorDistFront <= VIBR_ON_CM) near = true;
      if(d4 > 0 && d4 <= S4_VIBR_CM) near = true;
    }

    if(pulseActive && (nowMs - pulseStartMs >= MOTOR_PULSE_MS)){
      motorOff();
      pulseActive=false;
      nextAllowedPulseMs = nowMs + PULSE_REST_MS;
    }

    if(!pulseActive && near && (nowMs >= nextAllowedPulseMs)){
      motorOn();
      pulseActive=true;
      pulseStartMs = nowMs;
    }

    // DATA
    if(millis() - lastDataTxMs >= DATA_TX_INTERVAL_MS){
      lastDataTxMs = millis();

      BT.print("DATA t="); BT.print(secsSinceStart());
      BT.print(" s, S1="); if(d1<0) BT.print("NA"); else BT.print(d1,1);
      BT.print(" cm, S2="); if(d2<0) BT.print("NA"); else BT.print(d2,1);
      BT.print(" cm, S3="); if(d3<0) BT.print("NA"); else BT.print(d3,1);
      BT.print(" cm, S4="); if(d4<0) BT.print("NA"); else BT.print(d4,1);
      BT.print(" cm, MINf="); if(minD_filt<0) BT.print("NA"); else BT.print(minD_filt,1);

      BT.print(" cm, S3_TURN="); BT.print(s3TurnOpen?1:0);
      BT.print(", S3_APRX="); BT.print(s3CurrentApproach,1);

      BT.print(", S3_EVT=");
      if(s3EvtPulse==S3_EVT_START) BT.print("START");
      else if(s3EvtPulse==S3_EVT_END) BT.print("END");
      else BT.print("0");

      BT.println();
      s3EvtPulse = S3_EVT_NONE;
    }
  }

  // RX
  while(BT.available()){
    char c = (char)BT.read();
    if(c=='\r') continue;

    if(c=='\n'){
      btLine.trim();
      if(btLine.length()==0){ btLine=""; continue; }

      String cmdRaw = btLine;
      btLine = "";
      String cmdUp = cmdRaw;
      cmdUp.toUpperCase();

      if(cmdUp == "MOTOR OFF AHORA"){
        motorEnableUser=false;
        motorOff();
        pulseActive=false;
        nextAllowedPulseMs = millis();
        sendMsg("MOTOR DETENIDO POR EMERGENCIA");
        continue;
      }

      if(cmdUp == "CAL"){
        startRecalS3();
        continue;
      }

      if(commState == SESSION_RUNNING && sessionActive){
        if(cmdUp == "STOP"){
          // cerrar giro abierto
          if(s3TurnOpen && s3TurnCount < MAX_S3_TURN_EVENTS){
            unsigned long t_end = secsSinceStart()*1000UL;
            s3TurnEvents[s3TurnCount].t_start_ms = s3TurnStartMs;
            s3TurnEvents[s3TurnCount].t_end_ms   = t_end;
            s3TurnEvents[s3TurnCount].min_d3     = s3TurnMinD3;
            s3TurnEvents[s3TurnCount].max_approach_cm = s3TurnMaxApproach;
            s3TurnCount++;
            s3TurnOpen=false;
            s3CurrentApproach=0.0f;
          }

          unsigned long durS = secsSinceStart();
          sessionActive = false;
          motorOff();
          setBuzzerEnabled(false);

          // resumen completo
          printSessionSummary(durS);

          sendMsg("NUEVA SESION? SI/NO");
          commState = ASK_NEW_SESSION;
          continue;
        }

        if(cmdUp=="BUZZER ON"){ buzzerEnableUser=true;  sendMsg("BUZZER ENABLED"); continue; }
        if(cmdUp=="BUZZER OFF"){ buzzerEnableUser=false; setBuzzerEnabled(false); sendMsg("BUZZER DISABLED"); continue; }
        if(cmdUp=="MOTOR ON"){ motorEnableUser=true; sendMsg("MOTOR ENABLED"); continue; }
        if(cmdUp=="MOTOR OFF"){ motorEnableUser=false; motorOff(); pulseActive=false; sendMsg("MOTOR DISABLED"); continue; }
      }

      switch(commState){

        case ASK_PATIENT_ID: {
          restoreBaseParams(); 

          patientID = cmdRaw; patientID.trim();
          if(patientID.length()==0){
            sendMsg("ID PACIENTE?");
            break;
          }
          sendMsg("-------------------- " + patientID);
          sendMsg("Condicion? (1=sin nada, 2=buzzer, 3=buzzer+motor)");
          commState = ASK_CONDITION;
        } break;

        case ASK_CONDITION: {
          String s = cmdRaw; s.trim();
          conditionSelection = s;

          if(s=="1"){
            buzzerEnableUser=false; motorEnableUser=false;
            sendMsg("Condicion: SIN NADA.");
          } else if(s=="2"){
            buzzerEnableUser=true; motorEnableUser=false;
            sendMsg("Condicion: BUZZER SOLO.");
          } else if(s=="3"){
            buzzerEnableUser=true; motorEnableUser=true;
            sendMsg("Condicion: BUZZER + MOTOR.");
          } else {
            sendMsg("Respuesta no valida. Escriba 1,2 o 3.");
            break;
          }

          sendMsg("Cambiar PARAMETROS para esta sesion? (SI/NO)");
          commState = ASK_CHANGE_PARAMS;
        } break;

        case ASK_CHANGE_PARAMS:
          if(cmdUp=="NO"){
            printCurrentParams();
            s3Calibrated=false;
            sendMsg("Calibracion S3 (cabeza).");
            sendMsg("1) Cabeza recta. Escriba OK para medir 1 s.");
            commState = CAL_S3_BASE;
          }
          else if(cmdUp=="SI"){
            sendMsg("BUZZER FRONTAL inicio? (antes " + String(BEEP_START_CM,1) + " cm). Numero:");
            commState = SET_BEEP_START;
          } else sendMsg("Responda SI o NO.");
          break;

        case SET_BEEP_START: {
          float v;
          if(!parseFloatFromCmd(cmdRaw, v) || v < 5.0f || v > 300.0f){
            sendMsg("Valor no valido. Ej 50.");
            break;
          }
          BEEP_START_CM = v;
          sendMsg("BUZZER FRONTAL continuo? (antes " + String(BEEP_CONT_CM,1) + " cm). Numero:");
          commState = SET_BEEP_CONT;
        } break;

        case SET_BEEP_CONT: {
          float v;
          if(!parseFloatFromCmd(cmdRaw, v) || v < 1.0f || v >= BEEP_START_CM){
            sendMsg("No valido. Debe ser < inicio (" + String(BEEP_START_CM,1) + "). Ej 30.");
            break;
          }
          BEEP_CONT_CM = v;
          sendMsg("BUZZER LATERAL S4? (antes " + String(S4_BEEP_CM,1) + " cm). Numero:");
          commState = SET_S4_BEEP;
        } break;

        case SET_S4_BEEP: {
          float v;
          if(!parseFloatFromCmd(cmdRaw, v) || v < 1.0f || v > 300.0f){
            sendMsg("Valor no valido. Ej 15.");
            break;
          }
          S4_BEEP_CM = v;
          sendMsg("MOTOR FRONTAL umbral? (antes " + String(VIBR_ON_CM,1) + " cm). Numero:");
          commState = SET_VIBR_FRONT;
        } break;

        case SET_VIBR_FRONT: {
          float v;
          if(!parseFloatFromCmd(cmdRaw, v) || v < 1.0f || v > 300.0f){
            sendMsg("Valor no valido. Ej 22.");
            break;
          }
          VIBR_ON_CM = v;
          sendMsg("MOTOR LATERAL S4 umbral? (antes " + String(S4_VIBR_CM,1) + " cm). Numero:");
          commState = SET_S4_VIBR;
        } break;

        case SET_S4_VIBR: {
          float v;
          if(!parseFloatFromCmd(cmdRaw, v) || v < 1.0f || v > 300.0f){
            sendMsg("Valor no valido. Ej 6.");
            break;
          }
          S4_VIBR_CM = v;
          sendMsg("CRASH umbral? (antes " + String(CRASH_CM,1) + " cm). Numero (o 0=no tocar):");
          commState = SET_CRASH;
        } break;

        case SET_CRASH: {
          float v;
          if(!parseFloatFromCmd(cmdRaw, v)){
            sendMsg("Valor no valido. Numero o 0.");
            break;
          }
          if(v > 0.0f){
            if(v < 1.0f || v > 50.0f){
              sendMsg("Crash raro. Usa 4-10 o 0.");
              break;
            }
            CRASH_CM = v;
          }
          printCurrentParams();
          s3Calibrated=false;
          sendMsg("Calibracion S3 (cabeza).");
          sendMsg("1) Cabeza recta. Escriba OK para medir 1 s.");
          commState = CAL_S3_BASE;
        } break;

        case CAL_S3_BASE:
          if(cmdUp=="OK"){
            sendMsg("Midiendo S3 recto (1 s)...");
            s3BaseDist = calibrateS3For(1000);
            if(s3BaseDist < 0) sendMsg("Error: S3 sin lecturas. Repita OK.");
            else {
              sendMsg("Base S3 = " + String(s3BaseDist,1) + " cm");
              sendMsg("Gire cabeza izquierda. Escriba OK (1 s).");
              commState = CAL_S3_TURN;
            }
          } else sendMsg("Escriba OK.");
          break;

        case CAL_S3_TURN:
          if(cmdUp=="OK"){
            sendMsg("Midiendo S3 girado (1 s)...");
            s3TurnDist = calibrateS3For(1000);
            if(s3TurnDist < 0) sendMsg("Error: S3 sin lecturas. Repita OK.");
            else {
              float delta = fabs(s3BaseDist - s3TurnDist);
              s3DeltaThresh = delta * S3_TURN_FACTOR;
              s3Calibrated = true;

              sendMsg("Calibracion S3 OK. Umbral=" + String(s3DeltaThresh,1) + " cm");

              if(resumeToSession){
                sendMsg("Continuando sesion...");
                commState = SESSION_RUNNING;
                resumeToSession = false;
              } else {
                sendMsg("Escriba OK para empezar la sesion.");
                commState = WAIT_START_OK;
              }
            }
          } else sendMsg("Escriba OK.");
          break;

        case WAIT_START_OK:
          if(cmdUp=="OK"){
            resetSessionMetrics();
            sessionActive = true;
            sessionStartMs = millis();
            lastDataTxMs = 0;

            sendMsg("---- SESION INICIADA ----");
            sendMsg(String("Permisos iniciales -> Buzzer: ") + (buzzerEnableUser?"HABILITADO":"DESHABILITADO") +
                    String(" | Motor: ") + (motorEnableUser?"HABILITADO":"DESHABILITADO"));
            sendMsg("Use botones para Buzzer/Motor. STOP para terminar. (CAL para recalibrar)");
            commState = SESSION_RUNNING;
          } else sendMsg("Escriba OK.");
          break;

        case ASK_NEW_SESSION:
          if(cmdUp=="SI"){
            sendMsg("Condicion? (1=sin nada, 2=buzzer, 3=buzzer+motor)");
            commState = ASK_CONDITION;
          } else if(cmdUp=="NO"){
            resetSessionMetrics();
            patientID = "";
            conditionSelection = "";
            s3Calibrated=false;
            sendMsg("ID PACIENTE?");
            commState = ASK_PATIENT_ID;
          } else sendMsg("Responda SI o NO.");
          break;

        default: break;
      }

    } else {
      btLine += c;
      if(btLine.length() > 120) btLine.remove(0, btLine.length()-120);
    }
  }

  if(millis() - lastPromptMs >= PROMPT_PERIOD_MS){
    lastPromptMs = millis();
    if(commState == ASK_PATIENT_ID) sendMsg("ID PACIENTE?");
  }

  delay(10);
}
