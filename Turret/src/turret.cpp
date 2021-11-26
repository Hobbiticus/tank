#include <Arduino.h>

#include <WiFi.h>
//#include <esp_now.h>
#include "ESPNowW.h"
#include <ESP32Servo.h>
#include <esp32-hal-timer.h>
typedef void (*ITimer_Callback) ();
#define ARRAYSIZE(x) (sizeof(x) / sizeof(*x))


#define ENABLE_DEBUGGING
//#define USE_SENSORS
#define STAGE_COUNT 2
//#define CALIBRATION_MODE

const unsigned int LoaderReadyPos = 180;

struct VoltageToFireParams
{
  unsigned int m_Volts;
  unsigned char m_LoaderPos;
  unsigned int m_ChargeTime;
  unsigned int m_Stage1OnTime;
  unsigned int m_InterStageTime;
  unsigned int m_Stage2OnTime;
};

#ifdef CALIBRATION_MODE
VoltageToFireParams FireParams =
{
//  V,  pos, charge, 1-on,  inter, 2-on
   50,  30,  250000, 14000, 0,     4500
};
#define GET_FIRE_PARAM(VAR, PARAM) unsigned int VAR = FireParams.PARAM;

//this is the parameter you want to adjust with the left stick
unsigned int* LeftParam = &FireParams.m_Stage1OnTime;
//this is the parameter you want to adjust with the right stick
unsigned int* RightParam = &FireParams.m_Stage2OnTime;

#else

//fill this out!!
//right now calibrated for 140V!
VoltageToFireParams FireParams[4] =
{
//  V,  pos, charge, 1-on,  inter, 2-on
  {50,  30,  250000, 14000, 0,     6000},
  {80,  30,  500000, 10000, 0,     5500},
  {110, 40,  700000, 10000, 0,     5000},
  {140, 40, 1100000,  8000, 0,     4500}, //only one with 2-on that is valid
//  {170, ...
};
int GetFireParamsIndex(int voltage)
{
  for (int i = 0; i < ARRAYSIZE(FireParams) - 1; i++)
  {
    if (voltage < FireParams[i + 1].m_Volts)
      return i;
  }
  return ARRAYSIZE(FireParams) - 1;
}
#define GET_FIRE_PARAM(VAR, PARAM) \
  unsigned int VAR; \
  { \
    int paramIndex = GetFireParamsIndex(ChargeVoltage); \
    if (paramIndex == ARRAYSIZE(FireParams)-1) \
      VAR = FireParams[paramIndex].PARAM; \
    else \
      VAR = map(ChargeVoltage, FireParams[paramIndex].m_Volts, FireParams[paramIndex+1].m_Volts, FireParams[paramIndex].PARAM, FireParams[paramIndex+1].PARAM); \
  }


#endif

int ChargeVoltage = 140;

class IRQTimer
{
  private:
  const static int Frequency = 1000000; // 1 MHz (microseconds)
public:
  IRQTimer(uint8_t timer)
  {
    m_TimerIndex = timer;
    m_Timer = NULL;
    m_Callback = NULL;
  }

  void SetTimer(unsigned int intervalUS, ITimer_Callback callback, bool trigger_once)
  {
    if (m_Timer)
      timerEnd(m_Timer);
    m_Timer = timerBegin(m_TimerIndex, F_CPU / (Frequency * 3), true);
    m_Callback = callback;
    
    timerAttachInterrupt(m_Timer, m_Callback, true);
    timerAlarmWrite(m_Timer, intervalUS, !trigger_once);
    timerAlarmEnable(m_Timer);
  }

private:
  uint8_t m_TimerIndex;
  hw_timer_t* m_Timer;
  ITimer_Callback m_Callback;
};
IRQTimer ITimer(0);


const unsigned long LinkTimeoutUS = 2 * 1000 * 1000;

//#define PeerMAC "24:6F:28:17:C1:9C"
uint8_t PeerMac[6] = {0x24, 0x6F, 0x28, 0x17, 0xC1, 0x9C};
#ifdef ENABLE_DEBUGGING
uint8_t DebugMac[6] = {0x94, 0xB9, 0x7E, 0xDA, 0x01, 0x20};
#endif

void DebugPrint(String str)
{
    Serial.println(str);
#ifdef ENABLE_DEBUGGING
    esp_now_send(DebugMac, (uint8_t*)str.c_str(), strlen(str.c_str()) + 1);
#endif
}



#define SENSOR_PIN_1 33
#if STAGE_COUNT > 1 || defined(CALIBRATION_MODE)
#define SENSOR_PIN_2 25
#endif

#define COIL_PIN_1 19
#if STAGE_COUNT > 1
#define COIL_PIN_2 18
#endif

//design moved from 21 to 22 in Rev E
#define CHARGE_PIN 22

#define BUILTIN_LED 2
#define TILT_PIN 26
#define LOADER_PIN 27


//RC -> tank
struct StateMsg
{
  int16_t LeftSpeed;
  int16_t RightSpeed;
  int16_t TurretTurnSpeed;
  int16_t TurretTiltSpeed;
  uint8_t ChargePressed;
  uint8_t TriggerPressed;
  uint8_t PowerLevel;
};

//tank -> RC
struct StatusMsg
{
  uint8_t ChargeState;
};
StateMsg gState;

#define TILT_MIN 5
#define TILT_MAX 180

Servo servo;
Servo loader;


double gTiltPos = (TILT_MIN + TILT_MAX) / 2.0;
double MaxTiltSpeed = 75.0; //degrees per second


volatile int StartTime1 = 0;
volatile int SensorTime1 = 0;
volatile int StartTime2 = 0;
volatile int SensorTime2 = 0;
volatile int CurrentStage = 0;
volatile bool Complete = false;
volatile int LastTriggerTime = 0;
volatile int LastSensor1Time = 0;
volatile int LastSensor2Time = 0;
volatile int LastChargeTimeUS = 0;

volatile int LastRecvTimeUS = 0;
enum
{
  TS_IDLE = 0,
  TS_CHARGING,
  TS_CHARGED,
  TS_FIRING,
} TurretState = TS_IDLE;

volatile bool Shutdown = false;
//bool Charging = false;

void IRAM_ATTR TurnOffCharging(void)
{
  digitalWrite(CHARGE_PIN, LOW);
  TurretState = TS_CHARGED;
  DebugPrint("Charging complete");

  //tell the RC that we are charged!
  StatusMsg msg;
  msg.ChargeState = 2;
  esp_now_send(PeerMac, (uint8_t*)&msg, sizeof(msg));
}

#if STAGE_COUNT > 1
void IRAM_ATTR TurnOffStage2(void)
{
  digitalWrite(COIL_PIN_2, LOW);
  digitalWrite(BUILTIN_LED, LOW);
  DebugPrint("Turning off stage 2");
  TurretState = TS_IDLE;
  CurrentStage = 0;
  Complete = true;
}

void IRAM_ATTR TurnOnStage2(void)
{
  digitalWrite(COIL_PIN_2, HIGH);
  StartTime2 = micros();
#ifndef USE_SENSORS
  GET_FIRE_PARAM(stage2OnTime, m_Stage2OnTime);
  //unsigned int stage2OnTime = FireParams.m_Stage2OnTime;
  
  ITimer.SetTimer(stage2OnTime, TurnOffStage2, true);
  DebugPrint("Stage 2 on time = " + String(stage2OnTime) + " us");
#endif
  DebugPrint("Turned on stage 2");
}
#endif

void IRAM_ATTR TurnOffStage1(void)
{
  if (CurrentStage != 1)
    return;
  digitalWrite(COIL_PIN_1, LOW);
  DebugPrint("Turning off stage 1");

#if STAGE_COUNT > 1
  //start next stage
  CurrentStage = 2;
#ifdef CALIBRATION_MODE
  int delayTime = FireParams.m_InterStageTime;
#else
  int paramIndex = GetFireParamsIndex(ChargeVoltage);
  int delayTime = 0;
  if (paramIndex == ARRAYSIZE(FireParams)-1)
    delayTime = FireParams[paramIndex].m_InterStageTime;
  else
    delayTime = map(ChargeVoltage, FireParams[paramIndex].m_Volts, FireParams[paramIndex+1].m_Volts, FireParams[paramIndex].m_InterStageTime, FireParams[paramIndex+1].m_InterStageTime);
#endif
  
  if (delayTime > 0)
  {
    ITimer.SetTimer(delayTime, TurnOnStage2, true);
    DebugPrint("Delaying stage 2 by " + String(delayTime) + " us");
    return;
  }

  TurnOnStage2();
#else
  //no more stages - all done!
  digitalWrite(BUILTIN_LED, LOW);
  CurrentStage = 0;
  TurretState = TS_IDLE;
  Complete = true;
#endif
}

void OnTrigger()
{
  if (CurrentStage != 0)
    return;

  //debouncing
  int when = micros();
  if (when - LastTriggerTime < 1000 * 1000)
    return;
  LastTriggerTime = when;
  DebugPrint("Trigger!");

  //ALWAYS turn off charging
  digitalWrite(CHARGE_PIN, LOW);
  //return the loader to the ready position
  loader.write(LoaderReadyPos);
  
  //tell the RC to no longer illuminate the charge LED
  StatusMsg msg;
  msg.ChargeState = false;
  esp_now_send(PeerMac, (uint8_t*)&msg, sizeof(msg));

#if !defined(USE_SENSORS) && !defined(CALIBRATION_MODE)
  //figure out how long we need to run the first stage
  unsigned int stage1OnTime = 1000;
  int paramIndex = GetFireParamsIndex(ChargeVoltage);
  if (paramIndex == ARRAYSIZE(FireParams) - 1)
    stage1OnTime = FireParams[paramIndex].m_Stage1OnTime;
  else
    stage1OnTime = map(ChargeVoltage, FireParams[paramIndex].m_Volts, FireParams[paramIndex+1].m_Volts, FireParams[paramIndex].m_Stage1OnTime, FireParams[paramIndex+1].m_Stage1OnTime);
  DebugPrint("Stage1 on = " + String(stage1OnTime) + " us");
#endif
  
#ifdef CALIBRATION_MODE
  DebugPrint("Stage1 on = " + String(FireParams.m_Stage1OnTime) + " us, inter stage = " + String(FireParams.m_InterStageTime) + " us, Stage 2 delay = " + String(FireParams.m_Stage2OnTime) + " us");
  unsigned int stage1OnTime = FireParams.m_Stage1OnTime;

  //make sure our timing vars are reset
  SensorTime2 = SensorTime1 = 1;
#endif

  digitalWrite(BUILTIN_LED, HIGH);
  CurrentStage = 1;

  StartTime1 = micros();
  digitalWrite(COIL_PIN_1, HIGH);

#ifndef USE_SENSORS
  ITimer.SetTimer(stage1OnTime, TurnOffStage1, true);
#endif
}

void OnCharge()
{
  if (CurrentStage != 0)
    return;

  //debouncing
  int when = micros();
  if (when - LastChargeTimeUS < 200 * 1000)
    return;
  LastChargeTimeUS = when;
  DebugPrint("Charge!");

  if (TurretState == TS_CHARGING)
  {
    digitalWrite(CHARGE_PIN, LOW);
    TurretState = TS_CHARGED;
    Serial.println("Turning off charging because button was pressed again");
    return;
  }
  
  if (TurretState != TS_IDLE)
  {
    Serial.println("Not allowed to charge while not idle");
    return;
  }

  TurretState = TS_CHARGING;

  //tell the RC that we are charging!
  StatusMsg msg;
  msg.ChargeState = 1;
  esp_now_send(PeerMac, (uint8_t*)&msg, sizeof(msg));

#ifndef CALIBRATION_MODE
  ChargeVoltage = map(gState.PowerLevel, 0, 255, FireParams[0].m_Volts, FireParams[ARRAYSIZE(FireParams)-1].m_Volts);
  DebugPrint("Charge voltage = " + String(ChargeVoltage));

  //figure out how long we need to run the charger
  unsigned int chargeTime = 1000;
  int paramIndex = GetFireParamsIndex(ChargeVoltage);
  if (paramIndex == ARRAYSIZE(FireParams) - 1)
    chargeTime = FireParams[paramIndex].m_ChargeTime;
  else
    chargeTime = map(ChargeVoltage, FireParams[paramIndex].m_Volts, FireParams[paramIndex+1].m_Volts, FireParams[paramIndex].m_ChargeTime, FireParams[paramIndex+1].m_ChargeTime);
  DebugPrint("Charge time = " + String(chargeTime) + " us");
#endif

  //move the loader
  GET_FIRE_PARAM(loaderPos, m_LoaderPos)
  loader.write(loaderPos);
  DebugPrint("Loader pos = " + String(loaderPos));

  Serial.println("Charging ON");
  digitalWrite(CHARGE_PIN, HIGH);

#ifndef CALIBRATION_MODE
  ITimer.SetTimer(chargeTime, TurnOffCharging, true);
#endif
}

#ifdef CALIBRATION_MODE
void IRAM_ATTR SensorInterrupt1()
{
  int when = micros();
  if (when - LastSensor1Time < 200 * 1000)
    return;
  DebugPrint("Interrupt1! " + String(when));
  LastSensor1Time = when;
  SensorTime1 = when;
}

void IRAM_ATTR SensorInterrupt2()
{
  int when = micros();
  if (when - LastSensor2Time < 200 * 1000)
    return;
  DebugPrint("Interrupt2! " + String(when));
  LastSensor2Time = when;
  SensorTime2 = when;
}

#endif
#ifdef USE_SENSORS

void IRAM_ATTR SensorInterrupt1()
{
  if (CurrentStage != 1)
    return;

  int when = micros();
  if (when - LastSensor1Time < 200 * 1000)
    return;
  DebugPrint("Interrupt1!");
  LastSensor1Time = when;
  SensorTime1 = when;

  if (Delay_Stage1_Off > 0)
  {
    ITimer.SetTimer(Delay_Stage1_Off, TurnOffStage1, true);
    return;
  }
  TurnOffStage1();
}

#if STAGE_COUNT > 1
void IRAM_ATTR SensorInterrupt2()
{
  if (CurrentStage != 2)
    return;

  int when = micros();
  if (when - LastSensor2Time < 200 * 1000)
    return;
  DebugPrint("Interrupt2!");
  LastSensor2Time = when;
  SensorTime2 = when;

  if (Delay_Stage2_Off > 0)
  {
    ITimer.SetTimer(Delay_Stage2_Off, TurnOffStage2, true);
    return;
  }
  TurnOffStage2();
}
#endif

#endif

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent data!" : "Failed to send data");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  StateMsg* msg = (StateMsg*)incomingData;
  if (Shutdown)
  {
    Serial.println("....and we are back");
    Shutdown = false;
  }
  LastRecvTimeUS = micros();
  if (msg->LeftSpeed != gState.LeftSpeed)
  {
    gState.LeftSpeed = msg->LeftSpeed;
  }
  if (msg->RightSpeed != gState.RightSpeed)
  {
    gState.RightSpeed = msg->RightSpeed;
  }
  if (msg->TurretTurnSpeed != gState.TurretTurnSpeed)
  {
    gState.TurretTurnSpeed = msg->TurretTurnSpeed;
  }
  if (msg->TurretTiltSpeed != gState.TurretTiltSpeed)
  {
    gState.TurretTiltSpeed = msg->TurretTiltSpeed;
    Serial.println("Tilt speed is now " + String(gState.TurretTiltSpeed));
  }
  if (msg->PowerLevel != gState.PowerLevel)
  {
    gState.PowerLevel = msg->PowerLevel;
  }
#ifdef CALIBRATION_MODE
  //set the position of the loader based on the "power level"
  if (TurretState == TS_CHARGING || TurretState == TS_CHARGED)
  {
    FireParams.m_LoaderPos = map(gState.PowerLevel, 0, 255, 80, 5);
    DebugPrint("Load pos = " + String(FireParams.m_LoaderPos));
    loader.write(FireParams.m_LoaderPos);
  }
#endif
  if (msg->ChargePressed != gState.ChargePressed)
  {
    gState.ChargePressed = msg->ChargePressed;
    if (gState.ChargePressed)
    {
      //pressed the charge button - start charging! (and disable firing)
      Serial.println("CHARGE!!!");
      OnCharge();
    }
  }
  if (msg->TriggerPressed != gState.TriggerPressed)
  {
    gState.TriggerPressed = msg->TriggerPressed;
    if (gState.TriggerPressed && !gState.ChargePressed)
    {
      //pressed the trigger button - FIRE!!!!
      Serial.println("FIRE!!!");
      OnTrigger();
    }
  }
}

void DischargeCapacitors()
{
  digitalWrite(COIL_PIN_1, HIGH);
#if STAGE_COUNT > 1
  digitalWrite(COIL_PIN_2, HIGH);
#endif
  delay(1000);
  digitalWrite(COIL_PIN_1, LOW);
#if STAGE_COUNT > 1
  digitalWrite(COIL_PIN_2, LOW);
#endif
}

unsigned long LastUpdateTime = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(SENSOR_PIN_1, INPUT_PULLUP);
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW);
  pinMode(COIL_PIN_1, OUTPUT);
  digitalWrite(COIL_PIN_1, LOW);

#if STAGE_COUNT > 1 || defined(CALIBRATION_MODE)
  pinMode(SENSOR_PIN_2, INPUT_PULLUP);
#endif
#if STAGE_COUNT > 1
  pinMode(COIL_PIN_2, OUTPUT);
  digitalWrite(COIL_PIN_2, LOW);
#endif

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  Serial.begin(115200);
  Serial.println("Startup");

#if defined(USE_SENSORS) || defined(CALIBRATION_MODE)
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN_1), SensorInterrupt1, FALLING);
#if STAGE_COUNT > 1 || defined(CALIBRATION_MODE)
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN_2), SensorInterrupt2, FALLING);
#endif
#endif

  WiFi.mode(WIFI_MODE_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Failed to init ESP-NOW");
    return;
  }
  Serial.println(WiFi.macAddress());
  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, PeerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
#ifdef ENABLE_DEBUGGING
  memcpy(peerInfo.peer_addr, DebugMac, 6);
  esp_now_add_peer(&peerInfo);
#endif

  if (esp_now_register_recv_cb(OnDataRecv) != ESP_OK)
  {
    Serial.println("Failed to register receive callback\n");
    return;
  }
  Serial.println("ESP-NOW started");

  DebugPrint("Hello there!");

  gState.LeftSpeed = 0;
  gState.RightSpeed = 0;
  gState.TurretTurnSpeed = 0;
  gState.TurretTiltSpeed = 0;
  gState.ChargePressed = 0;
  gState.TriggerPressed = 0;
  LastUpdateTime = micros();

  servo.attach(TILT_PIN, 500, 2400);
  servo.write(gTiltPos);
  loader.attach(LOADER_PIN, 500, 2400);
  loader.write(LoaderReadyPos);
  DischargeCapacitors();
}

void UpdateServo()
{
  unsigned long now = micros();
  unsigned long elapsed = now - LastUpdateTime;
  LastUpdateTime = now;

  double movement = gState.TurretTiltSpeed * MaxTiltSpeed / 255000000.0 * elapsed;
  gTiltPos += movement;
  if (gTiltPos > TILT_MAX)
    gTiltPos = TILT_MAX;
  if (gTiltPos < TILT_MIN)
    gTiltPos = TILT_MIN;

  //Serial.println("Adding " + String(movement) + ", now " + String(gTiltPos));

  servo.write((int)gTiltPos);
}

#ifdef CALIBRATION_MODE
void UpdateStageDelay()
{
  static unsigned int LastUpdateTime = millis();
  if (millis() - LastUpdateTime < 1000)
    return;
  LastUpdateTime += 1000;

  if (gState.TurretTurnSpeed < -220)
    *RightParam += 1000;
  else if (gState.TurretTurnSpeed < -30)
    *RightParam += 250;
  else if (gState.TurretTurnSpeed > 220)
    *RightParam -= 1000;
  else if (gState.TurretTurnSpeed > 30)
    *RightParam -= 250;

  if (gState.LeftSpeed < -220)
    *LeftParam += 1000;
  else if (gState.LeftSpeed < -30)
    *LeftParam += 250;
  else if (gState.LeftSpeed > 220)
    *LeftParam -= 1000;
  else if (gState.LeftSpeed > 30)
    *LeftParam += 250;

  if (gState.TurretTurnSpeed < -30 || gState.TurretTurnSpeed > 30 || gState.LeftSpeed < -30 || gState.LeftSpeed > 30)
  {
    DebugPrint("left param = " + String(*LeftParam) + " us, right param = " + String(*RightParam) + " us");
  }
}
#endif

void loop() {
#ifdef CALIBRATION_MODE
  UpdateStageDelay();
#endif

  UpdateServo();
  if (Shutdown)
  {
    delay(500);
    return;
  }
  // put your main code here, to run repeatedly:
  int when = micros();

  //first, see if we went out of range and power down
  if (when - LastRecvTimeUS > LinkTimeoutUS)
  {
    Serial.println("Where did the transmitter go!??");
    Shutdown = true;
    //Charging = false;
    CurrentStage = 0;
    digitalWrite(COIL_PIN_1, LOW);
#if STAGE_COUNT > 1
    digitalWrite(COIL_PIN_2, LOW);
#endif
    digitalWrite(CHARGE_PIN, LOW);
  }

/*  if (CurrentStage != 0)
  {
    //make sure we don't run too long
    if (when - StartTime1 > 500 * 1000)
    {
      CurrentStage = 0;
      digitalWrite(COIL_PIN_1, LOW);
#if STAGE_COUNT > 1
      digitalWrite(COIL_PIN_2, LOW);
#endif
      digitalWrite(BUILTIN_LED, LOW);
      Serial.println("Coil running too long, safety triggered\n");
      DebugPrint("Coil running too long, safety triggered\n");
      DischargeCapacitors();
    }
  }*/

#ifdef CALIBRATION_MODE
  if (SensorTime2 > SensorTime1)
  {
    delay(500);
    Serial.println("Timed " + String(SensorTime2 - SensorTime1) + " us");
    DebugPrint("Timed " + String(SensorTime2 - SensorTime1) + " us");
    Complete = false;
    DischargeCapacitors();
    SensorTime2 = SensorTime1 = 0;
  }
#else
  if (Complete)
  {
    Complete = false;
    DischargeCapacitors();
  }
#endif
}
