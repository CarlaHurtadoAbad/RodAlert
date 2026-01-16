/* RodAlert MEGA 
   
   Final code with IMU monitoring:
   - 3 Ultrasonic Sensors
   - 2 IMUs
   - 1 Module Bluetooth
   - 1 Buzzer
   - 2 Buttons
   - 1 Motor ERM

*/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

// ===== CONFIGURACIÓN BLUETOOTH / SERIAL =====
#define BT Serial
const unsigned long BT_BAUD = 9600;

// ===== Pines =====
// FRONTAL ULTRASONIC SENSOR
#define TRIG1 13
#define ECHO1 12
#define TRIG2 11
#define ECHO2 10

// LATERAL ULTRASONIC SENSOR
#define TRIG3 22
#define ECHO3 24

// BUZZER + BUTTONS
#define BUZZER_PIN 7      
#define BUTTON_BUZZER 8   
#define BUTTON_MOTOR  9   

// L298N canal A (MOTOR)
const int L298_ENA = 6;   
const int L298_IN1 = 5;   
const int L298_IN2 = 4;   

// Distances/feedback
const unsigned long BEEP_INTERVAL_NEAR_MS = 100;
const unsigned long BEEP_INTERVAL_FAR_MS  = 600;

const float  BEEP_INT_CM  = 50.0f;   // S1 y S2: empieza a pitar hasta 50 cm
const float  BEEP_CONT_CM = 30.0f;   // S1 y S2: pitido continuo a ≤30 cm

// Lateral S3: lógica propia
const float  S3_BEEP_CM   = 15.0f;   // S3: buzzer continuo a ≤15 cm
const float  S3_VIBR_CM   = 6.0f;    // S3: vibración a ≤6 cm

const float VIBR_ON_CM  = 22.0f;  // frontal (S1/S2) vibración a ≤22 cm
const float VIBR_OFF_CM = 25.0f;  // histéresis 
const float CRASH_CM    = 6.0f;

const int   chosenPWM   = 255;     

// ===== Anti-parpadeo BUZZER =====
const unsigned long ON_DELAY_MS_BUZZER  = 30;
const unsigned long OFF_DELAY_MS_BUZZER = 450;
unsigned long buzzerCondSinceMs = 0, buzzerStopPendMs = 0;

// ===== Ultrasonidos “smart” =====
const unsigned long ULTRASONIC_TIMEOUT_US = 30000;
const unsigned int  INTERSHOT_US          = 1500;
const float         TOL_CM                = 3.0f;

// Filtro EMA
const bool  USE_EMA_DIST = false;   // DESACTIVADO para respuesta más rápida
const float EMA_ALPHA    = 0.7f;
float minD_filt = -1.0f;

// IMU DOBLE 
MPU6050 mpuHead(0x68), mpuChair(0x69);
const float GYRO_SCALE   = 131.0f;   // LSB/(º/s)
const uint16_t IMU_DT_MS = 20;
const float COMPLEMENT_ALPHA = 0.98f;

// Estado IMU
unsigned long imuLastMs = 0;
float yawH=0, yawC=0;
float pitchH=0, pitchC=0;
float gxBiasH=0, gxBiasC=0;
float yawRel=0, pitchRel=0;

// yaw limpio para análisis
float yawHeadAbs = 0.0f;   // cabeza vs mundo (corregida y normalizada)
float yawNeckRel = 0.0f;   // cabeza vs silla = wrapAngle(YHcorr - YCcorr)

// yaw simple de cabeza para debug
float calYawHead = 0.0f;

// SIGNOS para mapear el sentido con GX
float YAW_SIGN_H = -1.0f;   // cabeza
float YAW_SIGN_C = -1.0f;   // silla

float YAW_OFF_H  = 0.0f;
float YAW_OFF_C  = 0.0f;
bool  YDBG = false;
bool  H_is_head = true;
float dbgYH = 0.0f, dbgYC = 0.0f;

float headAnchor = 0.0f;
float prevYCcorr = 0.0f;
const float EPS_LEFT  = 0.4f;
const float EPS_RIGHT = 0.4f;

// Giros correctos hacia la izquierda 
const float YAW_EVENT_DEG = 15.0f;   // >= 15º izquierda
const float YAW_HYST_DEG  = 4.0f;    // histéresis (cierre al bajar)

struct GiroEvent {
  unsigned long t_start_ms;   // inicio del giro 
  unsigned long t_end_ms;     // fin del giro 
  float yaw_rel_max;          // pico máximo de yawRel durante el giro
};

const int MAX_GIRO_EVENTS = 60;
GiroEvent giroEvents[MAX_GIRO_EVENTS];
int giroCount = 0;
bool giroOpen = false;
float giroPeakRel = 0.0f;
unsigned long giroStartMs = 0;

static inline float rad2deg(float r){ return r*57.2957795f; }

// Normalizar ángulo a (-180, +180]
static inline float wrapAngle(float a){
  while (a > 180.0f)  a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

// FSM (Estados del Sistema)
enum CommState {
  ASK_PATIENT_ID,
  ASK_CONDITION,       // preguntar condición en vez de número de circuito
  ASK_DATE_DAY,        // pedir día
  ASK_DATE_MONTH,      // pedir mes
  ASK_DATE_YEAR,       // pedir año
  ASK_TIME_BEFORE_OK,  // pedir hora / M1 / TIME:...
  ASK_MOTOR_PRETEST,        // pregunta prueba 1 s (y repetir)
  MOTOR_PRETEST_RUNNING,    // vibración 1 s
  SHOW_STATUS_WAIT_OK,
  SESSION_RUNNING, SESSION_MENU_ASK_NEW, SESSION_MENU_ASK_FINALIZE
};
CommState commState = ASK_PATIENT_ID;

// Modo del estado SHOW_STATUS_WAIT_OK
enum ShowStatusMode { STATUS_NONE, STATUS_AFTER_CALIB, STATUS_AFTER_FINALIZE };
ShowStatusMode showStatusMode = STATUS_NONE;
bool calibrationDone = false;

// Sesión & métricas
bool sessionActive=false, sessionPaused=false;
unsigned long activeStartMs=0, accumulatedMs=0;

// Choques globales
int totalChoques=0; 
bool enChoque=false;
unsigned long tiemposChoques[60]; int idxChoques=0;

// Choques por sensor
unsigned long tiemposChoquesS1[60]; int idxChoquesS1=0;
unsigned long tiemposChoquesS2[60]; int idxChoquesS2=0;
unsigned long tiemposChoquesS3[60]; int idxChoquesS3=0;

// Eventos buzzer / motor
unsigned long tiemposMotorOn[120];  int idxMotorOn=0;
unsigned long tiemposBuzzerOn[120]; int idxBuzzerOn=0;

bool buzzerEnableUser=false, motorEnableUser=false;
bool buzzerIsOn=false;

// MÉTRICAS DE SEGURIDAD AÑADIDAS
float minSessionDist = -1.0f;      // distancia mínima global MINf
int   falseNegCount  = 0;          // choques sin aviso previo (3 s)
int   falsePosCount  = 0;          // avisos sin choque (5 s)
float sumDelayAvisoChoque = 0.0f;  // suma de (tChoque - tAviso)
int   nDelayAvisoChoque   = 0;     // nº de choques con aviso previo

// DATA
unsigned long lastDataTxMs=0;
const unsigned long DATA_TX_INTERVAL_MS=250;

String btLine="";
String patientID="";
String conditionSelection=""; // guarda "1","2","3" textual
String circuitNum="";         // por si quieres guardarlo
String fechaBase="", fechaHoraActual="", fechaHoraSesion="";

// RECORDATORIO AUTOMÁTICO (Timer)
unsigned long lastPromptMs=0;
const unsigned long PROMPT_PERIOD_MS=2000; 

// Array reservado
const int MAX_CORRECT = 120;
unsigned long correctTurnTimes[MAX_CORRECT]; int idxCorrect=0;

// Debug YAW
unsigned long lastYawDebugMs = 0;
const unsigned long YAW_DEBUG_INTERVAL_MS = 250;

// Utils
inline void sendMsg(const String& s){ Serial.println(s); }

unsigned long secsSinceStart(){
  unsigned long acc=accumulatedMs;
  if(sessionActive && !sessionPaused) acc+=(millis()-activeStartMs);
  return acc/1000UL;
}

// Ultrasonidos
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
  // Sensores apagados fuera de la sesión
  if (commState != SESSION_RUNNING) return -1.0f;

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

// IMU 
void imuBegin(){
  Wire.begin();
  Wire.setClock(100000);
  #if defined(WIRE_HAS_TIMEOUT) || defined(TWBR)
  Wire.setWireTimeout(3000, true);
  #endif
  mpuHead.initialize();
  mpuChair.initialize();
  bool okH = mpuHead.testConnection();
  bool okC = mpuChair.testConnection();
  if(!okH || !okC){
    sendMsg("ERROR: MPU6050 no detectados");
  } else sendMsg("✅ IMUs listos");
}

void imuCalibrate(){
  sendMsg("⚙️ Calibrando IMUs (3 s) en postura neutra...");
  const unsigned long T = 3000, dt=5;
  unsigned long t0 = millis();
  long gxSumH=0, gxSumC=0; 
  long n=0;
  while(millis()-t0 < T){
    int16_t ax,ay,az,gx,gy,gz;
    mpuHead.getMotion6(&ax,&ay,&az,&gx,&gy,&gz); gxSumH += gx;
    mpuChair.getMotion6(&ax,&ay,&az,&gx,&gy,&gz); gxSumC += gx;
    n++; delay(dt);
  }
  if(n == 0) n = 1;
  gxBiasH = (float)gxSumH / (float)n;
  gxBiasC = (float)gxSumC / (float)n;

  yawH=yawC=pitchH=pitchC=yawRel=pitchRel=0;
  calYawHead = 0.0f;
  giroCount=0; giroOpen=false; giroPeakRel=0.0f; giroStartMs=0;
  imuLastMs = millis();

  // reset nuevos yaw “limpios”
  yawHeadAbs = 0.0f;
  yawNeckRel = 0.0f;

  sendMsg("✅ IMU calibrado.");
}

void imuStep(){
  unsigned long now = millis();
  if(now - imuLastMs < IMU_DT_MS) return;
  float dt = (now - imuLastMs)*0.001f;
  imuLastMs = now;
  int16_t ax,ay,az,gx,gy,gz;

  // Cabeza
  mpuHead.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  float gxHdps = ((gx - gxBiasH) / GYRO_SCALE) * YAW_SIGN_H;
  yawH += gxHdps * dt;
  float pitchAccH = rad2deg(atan2f(-ax, sqrtf((float)ay*ay + (float)az*az)));
  pitchH = COMPLEMENT_ALPHA*(pitchH) + (1.0f-COMPLEMENT_ALPHA)*pitchAccH;

  // yaw simple de cabeza para debug
  calYawHead = yawH;

  // Silla
  mpuChair.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  float gxCdps = ((gx - gxBiasC) / GYRO_SCALE) * YAW_SIGN_C;
  yawC += gxCdps * dt;
  float pitchAccC = rad2deg(atan2f(-ax, sqrtf((float)ay*ay + (float)az*az)));
  pitchC = COMPLEMENT_ALPHA*(pitchC) + (1.0f-COMPLEMENT_ALPHA)*pitchAccC;

  float rawH = H_is_head ? yawH : yawC;
  float rawC = H_is_head ? yawC : yawH;
  float YHcorr = rawH + YAW_OFF_H;
  float YCcorr = rawC + YAW_OFF_C;
  dbgYH = YHcorr; 
  dbgYC = YCcorr;

  // Ángulos claros
  yawHeadAbs = wrapAngle(YHcorr);            // cabeza vs mundo
  yawNeckRel = wrapAngle(YHcorr - YCcorr);   // cuello vs silla

  float dYC = YCcorr - prevYCcorr;
  if (dYC > EPS_LEFT)        headAnchor = YHcorr;
  else if (dYC < -EPS_RIGHT) headAnchor = YHcorr;

  float yawUseful = YHcorr - headAnchor;
  yawRel = yawUseful;

  pitchRel = pitchH - pitchC;
  prevYCcorr = YCcorr;

  // Eventos de giro correcto hacia la izquierda (>= 15º) 
  if (sessionActive && !sessionPaused && commState == SESSION_RUNNING) {

    if (yawRel >= YAW_EVENT_DEG) {
      if (!giroOpen) {
        giroOpen    = true;
        giroStartMs = secsSinceStart()*1000UL;  // inicio del giro
        giroPeakRel = yawRel;                   // pico inicial
      } else if (yawRel > giroPeakRel) {
        giroPeakRel = yawRel;                   // actualiza pico
      }
    }
    else if (giroOpen && yawRel <= (YAW_EVENT_DEG - YAW_HYST_DEG)) {
      if (giroCount < MAX_GIRO_EVENTS) {
        unsigned long t_end = secsSinceStart()*1000UL;
        giroEvents[giroCount].t_start_ms = giroStartMs;
        giroEvents[giroCount].t_end_ms   = t_end;
        giroEvents[giroCount].yaw_rel_max = giroPeakRel;
        giroCount++;
      }
      giroOpen    = false;
      giroPeakRel = 0.0f;
      giroStartMs = 0;
    }

  } else {
    giroOpen    = false;
    giroPeakRel = 0.0f;
    giroStartMs = 0;
  }
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
    if(sessionActive && !sessionPaused && idxMotorOn<120)
      tiemposMotorOn[idxMotorOn++] = secsSinceStart();
    sendMsg("⚙️ MOTOR ON");
  }
}
void motorOff(){
  if(motorRunning){
    motorRunning=false;
    analogWrite(L298_ENA,0);
    sendMsg("🛑 MOTOR OFF");
  }
}

// Buzzer activo: +5V en el positivo, pin al negativo.
// LOW → suena, HIGH → no suena.
void setBuzzerEnabled(bool en){
  if(en && !buzzerIsOn){
    digitalWrite(BUZZER_PIN, LOW);   // ON
    buzzerIsOn = true;
    if(sessionActive && !sessionPaused && idxBuzzerOn<120)
      tiemposBuzzerOn[idxBuzzerOn++] = secsSinceStart();
    sendMsg("🔊 Buzzer ON");
  } else if(!en && buzzerIsOn){
    digitalWrite(BUZZER_PIN, HIGH);  // OFF
    buzzerIsOn = false;
    sendMsg("🔇 Buzzer OFF");
  }
}

// Fecha/Hora
String hhmmssFromMillis(){
  unsigned long totalSeconds=millis()/1000UL;
  unsigned long h=(totalSeconds/3600UL)%24;
  unsigned long m=(totalSeconds/60UL)%60;
  unsigned long s= totalSeconds%60;
  char b[9]; sprintf(b,"%02lu:%02lu:%02lu",h,m,s);
  return String(b);
}
void setTimeFromNow(){
  String t=hhmmssFromMillis();
  if(fechaBase.length()==0) fechaHoraActual="(sin-fecha) "+t;
  else fechaHoraActual = fechaBase+" "+t;
}
bool setTimeFromNowAlias(const String &cmdUp){
  if(cmdUp=="M1" || cmdUp=="TIME:NOW"){
    setTimeFromNow();
    sendMsg("🕒 " + fechaHoraActual);
    if(commState==ASK_TIME_BEFORE_OK || commState==SHOW_STATUS_WAIT_OK)
      sendMsg("ESCRIBA OK PARA CONTINUAR");
    return true;
  }
  return false;
}
bool setTimeFromText(const String& raw){
  String u=raw; u.trim(); if(!u.startsWith("TIME:")) return false;
  String p=u.substring(5); p.trim();
  if(p.equalsIgnoreCase("NOW")){ setTimeFromNow(); return true; }
  if(p.length()<5) return false;
  fechaHoraActual=p; 
  if(commState==ASK_TIME_BEFORE_OK || commState==SHOW_STATUS_WAIT_OK)
    sendMsg("ESCRIBA OK PARA CONTINUAR");
  return true;
}
bool setDateFromText(const String& raw){
  String u=raw; u.trim(); if(!u.startsWith("DATE:")) return false;
  fechaBase=u.substring(5); fechaBase.trim();
  sendMsg("📅 Fecha: "+fechaBase);
  return true;
}

// Lógica de pulsos del motor en sesión
const unsigned long MOTOR_PULSE_MS   = 1000;   // 1 s de vibración
const unsigned long PULSE_REST_MS    = 3000;   // 3 s de descanso
const unsigned long MOTOR_PRETEST_MS = 1000UL; // 1 s de vibración pretest

float calibYawTargetDeg = 0.5f;               // solo info en DATA (no se calibra)
unsigned long calibPulseMs = MOTOR_PULSE_MS;  // para DATA

bool pulseActive = false;
unsigned long pulseStartMs = 0;
unsigned long nextAllowedPulseMs = 0;

// prueba pre-sesión
unsigned long motorPretestStartMs = 0;

// datos de pretest
unsigned long lastPretestDataMs = 0;
const unsigned long PRETEST_DATA_INTERVAL_MS = 200;

// Setup
void setup(){
  pinMode(TRIG1,OUTPUT); pinMode(ECHO1,INPUT);
  pinMode(TRIG2,OUTPUT); pinMode(ECHO2,INPUT);
  pinMode(TRIG3,OUTPUT); pinMode(ECHO3,INPUT);

  pinMode(BUZZER_PIN,OUTPUT); 
  digitalWrite(BUZZER_PIN, HIGH); // buzzer OFF al inicio

  pinMode(BUTTON_BUZZER, INPUT_PULLUP);
  pinMode(BUTTON_MOTOR,  INPUT_PULLUP);

  pinMode(L298_ENA,OUTPUT); pinMode(L298_IN1,OUTPUT); pinMode(L298_IN2,OUTPUT);
  digitalWrite(L298_IN1,HIGH); 
  digitalWrite(L298_IN2,LOW);
  analogWrite(L298_ENA,0);

  BT.begin(BT_BAUD); 
  delay(200);

  BT.println("=== RodAlert boot ===");
  sendMsg("RodAlert (IMU+flujo) listo");

  imuBegin();

  lastPromptMs=millis(); 
  commState=ASK_PATIENT_ID;

  // Asegurar estado inactivo de actuadores
  buzzerEnableUser=false;
  motorEnableUser=false;
  setBuzzerEnabled(false);
  motorOff();
}

// Loop
void loop(){
  // ---------------- BOTONES ----------------
  static int lastBz = HIGH, lastMo = HIGH;  // reposo = HIGH
  static unsigned long lastBzChangeMs=0, lastMoChangeMs=0;
  unsigned long nowMs = millis();

  int rb = digitalRead(BUTTON_BUZZER);
  if(rb!=lastBz && (nowMs - lastBzChangeMs > 150)){
    lastBzChangeMs = nowMs;
    lastBz = rb;
    if(rb==LOW){   // pulsado
      buzzerEnableUser = !buzzerEnableUser;
      if(!buzzerEnableUser) setBuzzerEnabled(false);
      sendMsg(buzzerEnableUser? "BUZZER ENABLED":"BUZZER DISABLED");
    }
  }

  int rm = digitalRead(BUTTON_MOTOR);
  if(rm!=lastMo && (nowMs - lastMoChangeMs > 150)){
    lastMoChangeMs = nowMs;
    lastMo = rm;
    if(rm==LOW){   // pulsado
      motorEnableUser = !motorEnableUser;
      if(!motorEnableUser) motorOff();
      sendMsg(motorEnableUser? "MOTOR ENABLED":"MOTOR DISABLED");
    }
  }

  // Ultrasonidos (solo sesión)
  float d1=readUltrasonicSmart(TRIG1,ECHO1); 
  delayMicroseconds(1000);
  float d2=readUltrasonicSmart(TRIG2,ECHO2);
  delayMicroseconds(1000);
  float d3=readOnceCM(TRIG3,ECHO3);  // S3 ultrarrápido

  float minD=-1.0;
  if(d1>0 && d2>0 && d3>0)      minD = min(d1, min(d2,d3));
  else if(d1>0 && d2>0)         minD = min(d1,d2);
  else if(d1>0 && d3>0)         minD = min(d1,d3);
  else if(d2>0 && d3>0)         minD = min(d2,d3);
  else if(d1>0)                 minD = d1;
  else if(d2>0)                 minD = d2;
  else if(d3>0)                 minD = d3;

  if (USE_EMA_DIST) {
    if (minD >= 0) { 
      if (minD_filt < 0) minD_filt = minD; 
      else minD_filt = EMA_ALPHA*minD_filt + (1.0f-EMA_ALPHA)*minD; 
    }
  } else minD_filt = minD;

  // Distancia mínima global de la sesión
  if (sessionActive && !sessionPaused && minD_filt > 0) {
    if (minSessionDist < 0 || minD_filt < minSessionDist) {
      minSessionDist = minD_filt;
    }
  }

  // Choques (global + por sensor)
  if(sessionActive && !sessionPaused){
    bool choqueAhora = ((minD_filt>0 && minD_filt<=CRASH_CM) || (minD>0 && minD<=CRASH_CM));
    if(choqueAhora && !enChoque){
      enChoque=true; 
      totalChoques++;

      unsigned long tNow = secsSinceStart();

      if(idxChoques<60) tiemposChoques[idxChoques++] = tNow;

      bool c1 = (d1>0 && d1<=CRASH_CM);
      bool c2 = (d2>0 && d2<=CRASH_CM);
      bool c3 = (d3>0 && d3<=CRASH_CM);

      if(c1 && idxChoquesS1<60) tiemposChoquesS1[idxChoquesS1++] = tNow;
      if(c2 && idxChoquesS2<60) tiemposChoquesS2[idxChoquesS2++] = tNow;
      if(c3 && idxChoquesS3<60) tiemposChoquesS3[idxChoquesS3++] = tNow;

      sendMsg("⚠️ ¡Choque detectado!");
    }
    if(!choqueAhora && enChoque) enChoque = false;
  }

  imuStep();

  // Prueba de vibración rápida (1 s) PRE-SESIÓN
  if (commState == MOTOR_PRETEST_RUNNING) {
    unsigned long elapsed = millis() - motorPretestStartMs;

    if (millis() - lastPretestDataMs >= PRETEST_DATA_INTERVAL_MS) {
      lastPretestDataMs = millis();
      float t_s = elapsed / 1000.0f;
      BT.print("PRETEST t=");
      BT.print(t_s, 1);
      BT.print(" s, YAW_HEAD=");
      BT.print(yawH,1);
      BT.print(" deg, YAW_HEAD_REL=");
      BT.print(yawRel,1);
      BT.print(" deg, PITCH_HEAD=");
      BT.print(pitchH,1);
      BT.print(" deg, PITCH_CHAIR=");
      BT.print(pitchC,1);
      BT.println(" deg");
    }

    if (elapsed >= MOTOR_PRETEST_MS) {
      motorOff();
      sendMsg("Prueba de vibración de 1 s completada.");
      sendMsg("¿Desea repetir la prueba de vibración de 1 s? (SI/NO)");
      commState = ASK_MOTOR_PRETEST;
    }
  }

  // BUZZER SOLO EN SESIÓN
  static bool beepingNow=false; 
  static unsigned long lastBeepToggleMs=0;

  float buzzerDistFront = -1.0f;  // para S1/S2
  float buzzerDistLat   = -1.0f;  // para S3

  if (d1 > 0 && d1 <= BEEP_INT_CM) {
    buzzerDistFront = (buzzerDistFront < 0) ? d1 : min(buzzerDistFront, d1);
  }
  if (d2 > 0 && d2 <= BEEP_INT_CM) {
    buzzerDistFront = (buzzerDistFront < 0) ? d2 : min(buzzerDistFront, d2);
  }
  if (d3 > 0 && d3 <= S3_BEEP_CM) {
    buzzerDistLat = d3;
  }

  if (commState == SESSION_RUNNING) {
    if (buzzerEnableUser) {
      if (buzzerDistFront >= 0) {
        if (buzzerDistFront <= BEEP_CONT_CM) {
          if(buzzerCondSinceMs==0) buzzerCondSinceMs = nowMs;
          if(nowMs - buzzerCondSinceMs >= ON_DELAY_MS_BUZZER){
            setBuzzerEnabled(true); 
            buzzerStopPendMs = 0; 
          }
        } else {
          unsigned long interval = (buzzerDistFront <= (BEEP_CONT_CM+5.0f)) ? 
                                     BEEP_INTERVAL_NEAR_MS : 
                                     BEEP_INTERVAL_FAR_MS;
          if(nowMs - lastBeepToggleMs >= interval){
            lastBeepToggleMs = nowMs; 
            beepingNow = !beepingNow;
            if (beepingNow) setBuzzerEnabled(true); 
            else            setBuzzerEnabled(false);
          }
        }
      }
      else if (buzzerDistLat >= 0) {
        buzzerCondSinceMs = 0;
        beepingNow = false;
        setBuzzerEnabled(true);
        buzzerStopPendMs = 0;
      }
      else {
        buzzerCondSinceMs = 0;
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
      buzzerStopPendMs = 0;
      beepingNow = false;
      if (buzzerIsOn) setBuzzerEnabled(false);
    }
  } else {
    buzzerCondSinceMs = 0;
    buzzerStopPendMs = 0;
    beepingNow = false;
    if (buzzerIsOn) setBuzzerEnabled(false);
  }

  //  MOTOR EN SESIÓN (pulsos fijos 1s/3s)
  if (commState == SESSION_RUNNING) {
    float motorDistFront = -1.0f;
    if (d1 > 0) motorDistFront = (motorDistFront < 0) ? d1 : min(motorDistFront, d1);
    if (d2 > 0) motorDistFront = (motorDistFront < 0) ? d2 : min(motorDistFront, d2);

    bool near = false;
    if (motorEnableUser) {
      if (motorDistFront >= 0 && motorDistFront <= VIBR_ON_CM) {
        near = true;
      }
      if (d3 > 0 && d3 <= S3_VIBR_CM) {
        near = true;
      }
    }

    if (pulseActive && (nowMs - pulseStartMs >= MOTOR_PULSE_MS)) {
      motorOff(); 
      pulseActive = false;
      nextAllowedPulseMs = nowMs + PULSE_REST_MS;
    }

    if (!pulseActive && near && (nowMs >= nextAllowedPulseMs)) {
      motorOn(); 
      pulseActive = true; 
      pulseStartMs = nowMs;
      if(idxMotorOn < 120) tiemposMotorOn[idxMotorOn++] = secsSinceStart();
    }
  }

  // DATA SESIÓN
  if((commState==SESSION_RUNNING) && (millis() - lastDataTxMs >= DATA_TX_INTERVAL_MS)){
    lastDataTxMs=millis();
    BT.print("DATA t="); BT.print(secsSinceStart());
    BT.print(" s, S1=");
    if(d1<0) BT.print("NA"); else BT.print(d1,1);
    BT.print(" cm, S2=");
    if(d2<0) BT.print("NA"); else BT.print(d2,1);
    BT.print(" cm, S3=");
    if(d3<0) BT.print("NA"); else BT.print(d3,1);
    BT.print(" cm, MINf=");
    if(minD_filt<0) BT.print("NA"); else BT.print(minD_filt,1);
    BT.print(" cm, YAW_HEAD=");
    BT.print(yawH,1);
    BT.print(", YAW_HEAD_REL=");
    BT.print(yawRel,1);

    // Cabeza absoluta y cuello relativo normalizados
    BT.print(", YAW_HEAD_ABS=");
    BT.print(yawHeadAbs,1);
    BT.print(", YAW_NECK_REL=");
    BT.print(yawNeckRel,1);

    BT.print(", CAL_YAW=");
    BT.print(calibYawTargetDeg,1);
    BT.print(", CAL_PULSE_MS=");
    BT.println(calibPulseMs);
  }

  // RX (Serial/BT)
  while(BT.available()){
    char c=(char)BT.read(); 
    if(c=='\r') continue;
    if(c=='\n'){
      btLine.trim(); 
      if(btLine.length()==0){ btLine=""; continue; }
      String cmdRaw=btLine; 
      btLine="";
      String cmdUp=cmdRaw; 
      cmdUp.toUpperCase();

      // Fecha/hora
      if(setDateFromText(cmdRaw)) continue;
      if(setTimeFromNowAlias(cmdUp)) continue;
      if(setTimeFromText(cmdRaw)){
        sendMsg("🕒 "+fechaHoraActual);
        continue;
      }

      // COMANDOS UNIVERSALES
      if (cmdUp == "MOTOR OFF AHORA") {
        motorEnableUser = false;
        motorOff();
        pulseActive = false;
        nextAllowedPulseMs = millis();
        sendMsg("🛑 MOTOR DETENIDO POR EMERGENCIA");
        continue;
      }

      if (cmdUp == "CAL") {
        sendMsg("🔄 Recalibrando IMU (3 s). Mantenga postura neutra...");
        motorOff();
        setBuzzerEnabled(false);
        delay(200);

        imuCalibrate();
        headAnchor = 0;
        prevYCcorr = 0;

        giroCount   = 0;
        giroOpen    = false;
        giroPeakRel = 0.0f;
        giroStartMs = 0;

        sendMsg("✅ Recalibración completada.");
        continue;
      }

      if (cmdUp == "YDBG ON")  { YDBG = true;  sendMsg("YDBG=ON"); continue; }
      if (cmdUp == "YDBG OFF") { YDBG = false; sendMsg("YDBG=OFF"); continue; }
      if (cmdUp == "YSWAP")    { 
        H_is_head = !H_is_head; 
        sendMsg(H_is_head ? "HEAD=0x68, CHAIR=0x69" : "HEAD=0x69, CHAIR=0x68"); 
        continue;
      }
      if (cmdUp == "YZERO")    {
        float rawH = H_is_head ? yawH : yawC;
        float rawC = H_is_head ? yawC : yawH;
        YAW_OFF_H = -rawH; 
        YAW_OFF_C = -rawC;
        headAnchor = 0; 
        prevYCcorr = 0;
        sendMsg("Offsets a cero."); 
        continue;
      }

      switch(commState){
        case ASK_PATIENT_ID:
          if(cmdRaw.length()>0){
            patientID=cmdRaw; 
            sendMsg("-------------------- "+patientID);
            sendMsg("¿Qué tipo de condición desea? (escriba 1, 2 o 3)");
            sendMsg("1) Sin nada");
            sendMsg("2) Con Buzzer solo");
            sendMsg("3) Con Buzzer y motor");
            commState=ASK_CONDITION;
          }
          break;

        case ASK_CONDITION:
          if(cmdRaw.length()>0){
            {
              String s = cmdRaw; s.trim();
              conditionSelection = s;
              if(s == "1"){
                buzzerEnableUser = false;
                motorEnableUser = false;
                sendMsg("Condición: SIN NADA. Buzzer y Motor deshabilitados inicialmente.");
              } else if(s == "2"){
                buzzerEnableUser = true;
                motorEnableUser = false;
                sendMsg("Condición: BUZZER SOLO. Buzzer habilitado inicialmente; motor deshabilitado.");
              } else if(s == "3"){
                buzzerEnableUser = true;
                motorEnableUser = true;
                sendMsg("Condición: BUZZER y MOTOR habilitados inicialmente.");
              } else {
                sendMsg("Respuesta no válida. Escriba 1, 2 o 3.");
                break;
              }
            }
            sendMsg("Introduzca DÍA (número):");
            commState = ASK_DATE_DAY;
          }
          break;

        case ASK_DATE_DAY:
          if(cmdRaw.length()>0){
            {
              String d = cmdRaw; d.trim();
              fechaBase = "";
              fechaBase += (d.length()==1 ? "0"+d : d);
            }
            sendMsg("Introduzca MES (número):");
            commState = ASK_DATE_MONTH;
          }
          break;

        case ASK_DATE_MONTH:
          if(cmdRaw.length()>0){
            {
              String m = cmdRaw; m.trim();
              fechaBase += "-";
              fechaBase += (m.length()==1 ? "0"+m : m);
            }
            sendMsg("Introduzca AÑO (4 dígitos):");
            commState = ASK_DATE_YEAR;
          }
          break;

        case ASK_DATE_YEAR:
          if(cmdRaw.length()>0){
            {
              String y = cmdRaw; y.trim();
              fechaBase += "-";
              fechaBase += y;
            }
            sendMsg("📅 Fecha establecida: " + fechaBase);
            sendMsg("Introduzca HORA (HH:MM:SS) o escriba M1 para usar hora actual:");
            commState = ASK_TIME_BEFORE_OK;
          }
          break;

        case ASK_TIME_BEFORE_OK:
          if(cmdUp=="OK"){
            if(fechaHoraActual.length()==0){ 
              sendMsg("Primero M1/TIME:NOW o TIME:HH:MM:SS"); 
              break; 
            }
            motorOff(); 
            setBuzzerEnabled(false);
            imuCalibrate();      
            headAnchor = 0; 
            prevYCcorr = 0;

            calibrationDone = true;
            fechaHoraSesion = fechaHoraActual;

            sendMsg("Calibracion IMU completa.");
            sendMsg("Motor: OFF | Buzzer: OFF.");
            sendMsg("¿Desea realizar prueba de vibración rápida de 1 s antes de la sesión? (SI/NO)");
            commState = ASK_MOTOR_PRETEST;
            showStatusMode = STATUS_AFTER_CALIB;
          } else {
            if(setTimeFromNowAlias(cmdUp)) {
              continue;
            }
            if(setTimeFromText(cmdRaw)){
              sendMsg("🕒 "+fechaHoraActual);
              continue;
            }
            String maybeTime = cmdRaw; maybeTime.trim();
            if(maybeTime.length() >= 5 && maybeTime.indexOf(':')>0){
              fechaHoraActual = fechaBase + " " + maybeTime;
              sendMsg("🕒 "+fechaHoraActual);
              sendMsg("ESCRIBA OK PARA CONTINUAR");
            } else {
              sendMsg("Escriba M1 para hora actual, TIME:HH:MM:SS, TIME:NOW o introduzca hora HH:MM:SS. Después escriba OK.");
            }
          }
          break;

        case ASK_MOTOR_PRETEST:
          if (cmdUp == "SI") {
            sendMsg("Iniciando prueba de vibración de 1 s. Use STOP para cancelarla.");
            motorOn();
            motorPretestStartMs = millis();
            lastPretestDataMs = 0;
            commState = MOTOR_PRETEST_RUNNING;
          } else if (cmdUp == "NO") {
            sendMsg("Prueba de vibración finalizada.");
            sendMsg("Escriba OK para empezar la sesion.");
            commState = SHOW_STATUS_WAIT_OK;
          } else {
            sendMsg("Responda SI para iniciar/repetir la prueba de 1 s o NO para continuar.");
          }
          break;

        case MOTOR_PRETEST_RUNNING:
          if (cmdUp == "STOP") {
            motorOff();
            sendMsg("Prueba de vibración detenida por el usuario.");
            sendMsg("¿Desea repetir la prueba de vibración de 1 s? (SI/NO)");
            commState = ASK_MOTOR_PRETEST;
          } else {
            sendMsg("Prueba en curso. Use STOP para detener, o espere a que termine.");
          }
          break;

        case SHOW_STATUS_WAIT_OK:
          if(cmdUp=="OK"){
            if(showStatusMode == STATUS_AFTER_CALIB && calibrationDone){
              setBuzzerEnabled(false);
              motorOff();
              sessionActive=true; 
              sessionPaused=false;
              accumulatedMs=0; 
              activeStartMs=millis();

              totalChoques=0; 
              idxChoques=0;
              idxChoquesS1=idxChoquesS2=idxChoquesS3=0;
              enChoque=false;
              idxMotorOn=0; idxBuzzerOn=0; idxCorrect=0;
              giroCount=0; giroOpen=false; giroPeakRel=0.0f; giroStartMs=0;
              minD_filt=-1.0f; 
              lastDataTxMs=0;
              pulseActive=false; 
              nextAllowedPulseMs=millis();

              // reset seguridad
              minSessionDist = -1.0f;
              falseNegCount  = 0;
              falsePosCount  = 0;
              sumDelayAvisoChoque = 0.0f;
              nDelayAvisoChoque   = 0;

              sendMsg("---- SESION INICIADA ----");
              String perms = String("Permisos iniciales -> Buzzer: ") + (buzzerEnableUser ? "HABILITADO" : "DESHABILITADO") +
                             String(" | Motor: ") + (motorEnableUser ? "HABILITADO" : "DESHABILITADO");
              sendMsg(perms);
              sendMsg("Use los botones para cambiar Buzzer/Motor en cualquier momento.");
              sendMsg("STOP para terminar");
              commState=SESSION_RUNNING;
              showStatusMode = STATUS_NONE;
            } else if(showStatusMode == STATUS_AFTER_FINALIZE){
              buzzerEnableUser=false; 
              motorEnableUser=false;
              setBuzzerEnabled(false); 
              motorOff();
              sessionActive=false; 
              sessionPaused=false;
              accumulatedMs=0; 
              activeStartMs=0;
              totalChoques=0; 
              idxChoques=0;
              idxChoquesS1=idxChoquesS2=idxChoquesS3=0;
              enChoque=false;
              idxMotorOn=0; idxBuzzerOn=0; idxCorrect=0;
              giroCount=0; giroOpen=false; giroPeakRel=0.0f; giroStartMs=0;
              minD_filt=-1.0f; 
              lastDataTxMs=0;

              minSessionDist = -1.0f;
              falseNegCount  = 0;
              falsePosCount  = 0;
              sumDelayAvisoChoque = 0.0f;
              nDelayAvisoChoque   = 0;

              sendMsg("ID PACIENTE?");
              commState=ASK_PATIENT_ID;
              showStatusMode = STATUS_NONE;
              lastPromptMs = millis();
            } else {
              sendMsg("OK recibido.");
            }
          }
          break;

        case SESSION_RUNNING:
          if(cmdUp=="STOP"){
            if(sessionActive && !sessionPaused){ 
              accumulatedMs += (millis()-activeStartMs); 
              sessionPaused=true; 
            }
            unsigned long durS=accumulatedMs/1000UL;
            sendMsg("---- RESUMEN DE LA SESION ----");
            sendMsg("Duración: "+String(durS)+" s");

            // Distancia mínima global
            if (minSessionDist > 0) {
              sendMsg("Distancia mínima registrada (MINf): " + String(minSessionDist,1) + " cm");
            } else {
              sendMsg("Distancia mínima registrada (MINf): NA");
            }

            // Choques globales
            sendMsg("Choques totales (cualquier sensor): " + String(totalChoques));
            for(int i=0;i<idxChoques;i++){
              sendMsg("  Choque_global #" + String(i+1) + " en t=" + String(tiemposChoques[i]) + " s");
            }

            // Choques por sensor
            sendMsg("Choques por sensor:");
            sendMsg("  S1 frontal: " + String(idxChoquesS1));
            for(int i=0;i<idxChoquesS1;i++){
              sendMsg("    Choque_S1 #" + String(i+1) + " en t=" + String(tiemposChoquesS1[i]) + " s");
            }
            sendMsg("  S2 frontal: " + String(idxChoquesS2));
            for(int i=0;i<idxChoquesS2;i++){
              sendMsg("    Choque_S2 #" + String(i+1) + " en t=" + String(tiemposChoquesS2[i]) + " s");
            }
            sendMsg("  S3 lateral: " + String(idxChoquesS3));
            for(int i=0;i<idxChoquesS3;i++){
              sendMsg("    Choque_S3 #" + String(i+1) + " en t=" + String(tiemposChoquesS3[i]) + " s");
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

            // Giros correctos hacia la izquierda
            sendMsg("Giros correctos hacia la izquierda (>=15 deg): " + String(giroCount));
            for (int i = 0; i < giroCount; i++) {
              float t_ini_s = giroEvents[i].t_start_ms / 1000.0f;
              float t_fin_s = giroEvents[i].t_end_ms   / 1000.0f;
              float pico    = giroEvents[i].yaw_rel_max;

              String line = "  Giro_izq #";
              line += String(i + 1);
              line += " de t=";
              line += String(t_ini_s, 1);
              line += " s a t=";
              line += String(t_fin_s, 1);
              line += " s (pico=";
              line += String(pico, 1);
              line += " deg)";
              sendMsg(line);
            }

            // CÁLCULO DE FALSOS NEGATIVOS / POSITIVOS Y TIEMPO AVISO→CHOQUE 
            falseNegCount        = 0;
            falsePosCount        = 0;
            sumDelayAvisoChoque  = 0.0f;
            nDelayAvisoChoque    = 0;

            bool hayAsistencia = (conditionSelection != "1"); // 1 = sin nada

            if (hayAsistencia) {
              // Falsos negativos + delay aviso = choque
              for (int i = 0; i < idxChoques; i++) {
                unsigned long tC = tiemposChoques[i];
                bool hadWarning   = false;
                float lastWarn    = -1.0f;

                // Buzzer en ventana [tC-3, tC]
                for (int j = 0; j < idxBuzzerOn; j++) {
                  float tB = (float)tiemposBuzzerOn[j];
                  if (tB >= (float)tC - 2.0f && tB <= (float)tC) {
                    hadWarning = true;
                    if (tB > lastWarn) lastWarn = tB;
                  }
                }
                // Motor en ventana [tC-3, tC]
                for (int j = 0; j < idxMotorOn; j++) {
                  float tM = (float)tiemposMotorOn[j];
                  if (tM >= (float)tC - 3.0f && tM <= (float)tC) {
                    hadWarning = true;
                    if (tM > lastWarn) lastWarn = tM;
                  }
                }

                if (!hadWarning) {
                  falseNegCount++;
                } else if (lastWarn >= 0.0f) {
                  sumDelayAvisoChoque += ((float)tC - lastWarn);
                  nDelayAvisoChoque++;
                }
              }

              // Falsos positivos: avisos sin choque en los 5 s siguientes
              // Buzzer
              for (int j = 0; j < idxBuzzerOn; j++) {
                float tB = (float)tiemposBuzzerOn[j];
                bool choqueAfter = false;
                for (int i = 0; i < idxChoques; i++) {
                  float tC = (float)tiemposChoques[i];
                  if (tC >= tB && tC <= tB + 5.0f) {
                    choqueAfter = true;
                    break;
                  }
                }
                if (!choqueAfter) falsePosCount++;
              }
              // Motor
              for (int j = 0; j < idxMotorOn; j++) {
                float tM = (float)tiemposMotorOn[j];
                bool choqueAfter = false;
                for (int i = 0; i < idxChoques; i++) {
                  float tC = (float)tiemposChoques[i];
                  if (tC >= tM && tC <= tM + 5.0f) {
                    choqueAfter = true;
                    break;
                  }
                }
                if (!choqueAfter) falsePosCount++;
              }
            }

            if (hayAsistencia) {
              sendMsg("Choques sin aviso previo en 3 s (falsos negativos): " + String(falseNegCount));
              sendMsg("Avisos sin choque en 5 s (falsos positivos): " + String(falsePosCount));
              if (nDelayAvisoChoque > 0) {
                float delayMedio = sumDelayAvisoChoque / (float)nDelayAvisoChoque;
                sendMsg("Tiempo medio aviso→choque: " + String(delayMedio, 2) + " s (n=" + String(nDelayAvisoChoque) + ")");
              } else {
                sendMsg("Tiempo medio aviso→choque: NA (sin choques con aviso previo)");
              }
            } else {
              sendMsg("Condición SIN ASISTENCIA: no se calculan falsos positivos/negativos.");
            }

            sendMsg("NUEVA SESION? SI/NO");
            commState=SESSION_MENU_ASK_NEW;
            lastPromptMs = millis();
          } else if(cmdUp=="BUZZER ON"){ 
            buzzerEnableUser=true; 
            if(commState==SESSION_RUNNING) setBuzzerEnabled(true);
          } else if(cmdUp=="BUZZER OFF"){ 
            buzzerEnableUser=false; setBuzzerEnabled(false);
          } else if(cmdUp=="MOTOR ON"){ 
            motorEnableUser=true;
          } else if(cmdUp=="MOTOR OFF"){ 
            motorEnableUser=false; motorOff(); pulseActive=false; 
          }
          break;

        case SESSION_MENU_ASK_NEW:
          if(cmdUp=="SI"){
            buzzerEnableUser=false; setBuzzerEnabled(false);
            motorEnableUser=false;  motorOff();
            sessionActive=false; sessionPaused=false;
            accumulatedMs=0; activeStartMs=0;
            totalChoques=0; idxChoques=0;
            idxChoquesS1=idxChoquesS2=idxChoquesS3=0;
            enChoque=false;
            giroCount=0; giroOpen=false; giroPeakRel=0.0f; giroStartMs=0;
            idxMotorOn=0; idxBuzzerOn=0; idxCorrect=0;
            patientID=""; conditionSelection=""; circuitNum="";

            minSessionDist = -1.0f;
            falseNegCount  = 0;
            falsePosCount  = 0;
            sumDelayAvisoChoque = 0.0f;
            nDelayAvisoChoque   = 0;

            sendMsg("---- NUEVA CONFIGURACION ----");
            sendMsg("ID PACIENTE?");
            commState=ASK_PATIENT_ID;
            lastPromptMs = millis();
          } else if(cmdUp=="NO"){
            sendMsg("FINALIZAR SESION ACTUAL? SI/NO");
            commState=SESSION_MENU_ASK_FINALIZE;
            lastPromptMs = millis(); 
          }
          break;

        case SESSION_MENU_ASK_FINALIZE:
          if(cmdUp=="SI"){
            sessionActive=false; sessionPaused=false;
            sendMsg("ESCRIBA OK PARA REINICIAR");
            commState=SHOW_STATUS_WAIT_OK;
            showStatusMode = STATUS_AFTER_FINALIZE;
          } else if(cmdUp=="NO"){
            if(sessionActive && sessionPaused){ 
              sessionPaused=false; 
              activeStartMs=millis(); 
            }
            sendMsg("Sesion reanudada."); 
            commState=SESSION_RUNNING;
          }
          break;
      }
    } else {
      btLine += c;
      if(btLine.length() > 120) 
        btLine.remove(0, btLine.length()-120);
    }
  }

  // RECORDATORIO ID PACIENTE
  if(millis() - lastPromptMs >= PROMPT_PERIOD_MS) {
    lastPromptMs = millis();
    if (commState == ASK_PATIENT_ID) {
      sendMsg("ID PACIENTE?");
    }
  }

  delay(10);
}
