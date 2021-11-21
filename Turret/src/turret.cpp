#include <Arduino.h>

#include <WiFi.h>
//#include <esp_now.h>
#include "ESPNowW.h"
#include <ESP32Servo.h>
#include <esp32-hal-timer.h>
typedef void (*ITimer_Callback) ();

#define ENABLE_DEBUGGING
#define USE_SENSORS
#define STAGE_COUNT 1

//...in microseconds
#ifdef USE_SENSORS
const unsigned int Delay_Stage1_Off = 0;
const unsigned int Delay_Stage2_Off = 0;
#endif
unsigned int Delay_Stage2_On = 0;

#ifndef USE_SENSORS
struct VoltageToTiming
{
  unsigned int m_Volts;
  unsigned int m_Time;
};
//fill this out!!
VoltageToTiming Stage1OnTiming[5] =
{
  {50, 18000}, //18000 thick, //5000 - 18000 thin
  {80, 1000},
  {110, 1000},
  {140, 1000},
  {170, 1000}
};

unsigned int Stage1OnTime = 5000;
#if STAGE_COUNT > 1
unsigned int Stage2OnTime = 15000;
#endif
#endif

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

#define ARRAYSIZE(x) (sizeof(x) / sizeof(*x))

const unsigned long LinkTimeoutUS = 2 * 1000 * 1000;

//#define PeerMAC "24:6F:28:17:C1:9C"
uint8_t PeerMac[6] = {0x24, 0x6F, 0x28, 0x17, 0xC1, 0x9C};
#ifdef ENABLE_DEBUGGING
uint8_t DebugMac[6] = {0x94, 0xB9, 0x7E, 0xDA, 0x01, 0x20};

void DebugPrint(String str)
{
    esp_now_send(DebugMac, (uint8_t*)str.c_str(), strlen(str.c_str()) + 1);
}
#else
#define DebugPrint(x)
#endif



#define SENSOR_PIN_1 33
#if STAGE_COUNT > 1
#define SENSOR_PIN_2 25
#endif

#define COIL_PIN_1 19
#if STAGE_COUNT > 1
#define COIL_PIN_2 18
#endif

//design moved from 21 to 22 in Rev E
#define CHARGE_PIN 22

#define BUILTIN_LED 2
//RX on logic level shifters!!
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
volatile bool Shutdown = false;
bool Charging = false;

#if STAGE_COUNT > 1
void IRAM_ATTR TurnOffStage2(void)
{
  digitalWrite(COIL_PIN_2, LOW);
  digitalWrite(BUILTIN_LED, LOW);
  DebugPrint("Turning off stage 2");
  CurrentStage = 0;
  Complete = true;
}

void IRAM_ATTR TurnOnStage2(void)
{
  digitalWrite(COIL_PIN_2, HIGH);
  StartTime2 = micros();
#ifndef USE_SENSORS
  ITimer.SetTimer(Stage2OnTime, TurnOffStage2, true);
#endif
  DebugPrint("Turning on stage 2");
}
#endif

void IRAM_ATTR TurnOffStage1(void)
{
  digitalWrite(COIL_PIN_1, LOW);

#if STAGE_COUNT > 1
  //start next stage
  CurrentStage = 2;
  if (Delay_Stage2_On > 0)
  {
    ITimer.SetTimer(Delay_Stage2_On, TurnOnStage2, true);
    DebugPrint("Turning off stage 1");
    return;
  }

  DebugPrint("Turning off stage 1");
  TurnOnStage2();
#else
  //no more stages - all done!
  DebugPrint("Turning off stage 1");
  digitalWrite(BUILTIN_LED, LOW);
  CurrentStage = 0;
  Complete = true;
#endif
}

void OnTrigger()
{
  if (CurrentStage != 0)
    return;

  int when = micros();
  if (when - LastTriggerTime < 1000 * 1000)
    return;

  if (Charging)
  {
    Serial.println("Charging OFF");
    Charging = false;
    digitalWrite(CHARGE_PIN, LOW);
  }
  loader.write(180);
  
  StatusMsg msg;
  msg.ChargeState = false;
  esp_now_send(PeerMac, (uint8_t*)&msg, sizeof(msg));

  DebugPrint("Trigger!");
  LastTriggerTime = when;
  Serial.println("Trigger pushed");
  digitalWrite(BUILTIN_LED, HIGH);
  CurrentStage = 1;
  digitalWrite(CHARGE_PIN, LOW);

  StartTime1 = micros();
  digitalWrite(COIL_PIN_1, HIGH);

#ifndef USE_SENSORS
  ITimer.SetTimer(Stage1OnTime, TurnOffStage1, true);
#endif
}

void OnCharge()
{
  if (CurrentStage != 0)
    return;

  Charging = !Charging;
  
  if (Charging)
  {
    StatusMsg msg;
    msg.ChargeState = Charging;
    esp_now_send(PeerMac, (uint8_t*)&msg, sizeof(msg));
    loader.write(5);
  }
  
  if (!Charging)
  {
    Serial.println("Charging OFF");
    digitalWrite(CHARGE_PIN, LOW);
    return;
  }
  Serial.println("Charging ON");
  LastChargeTimeUS = micros();
  digitalWrite(CHARGE_PIN, HIGH);
}

void IRAM_ATTR SensorInterrupt1()
{
#ifdef USE_SENSORS
  if (CurrentStage != 1)
    return;
#endif
  int when = micros();
  if (when - LastSensor1Time < 200 * 1000)
    return;
  DebugPrint("Interrupt1!");
  LastSensor1Time = when;
  SensorTime1 = when;

#ifdef USE_SENSORS
  if (Delay_Stage1_Off > 0)
  {
    ITimer.SetTimer(Delay_Stage1_Off, TurnOffStage1, true);
    return;
  }
  TurnOffStage1();
#endif
}

#if STAGE_COUNT > 1
void IRAM_ATTR SensorInterrupt2()
{
#ifdef USE_SENSORS
  if (CurrentStage != 2)
    return;
#endif
  int when = micros();
  if (when - LastSensor2Time < 200 * 1000)
    return;
  LastSensor2Time = when;
  SensorTime2 = when;

#ifdef USE_SENSORS
  if (Delay_Stage2_Off > 0)
  {
    ITimer.SetTimer(Delay_Stage2_Off, TurnOffStage2, true);
    return;
  }
  TurnOffStage2();
#endif
}
#endif


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent data!" : "Failed to send data");
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

#if STAGE_COUNT > 1
  pinMode(SENSOR_PIN_2, INPUT_PULLUP);
  pinMode(COIL_PIN_2, OUTPUT);
  digitalWrite(COIL_PIN_2, LOW);
#endif

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  Serial.begin(115200);
  Serial.println("Startup");

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN_1), SensorInterrupt1, FALLING);
#if STAGE_COUNT > 1
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN_2), SensorInterrupt2, FALLING);
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
  loader.write(180);
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

void loop() {
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
    Charging = false;
    CurrentStage = 0;
    digitalWrite(COIL_PIN_1, LOW);
#if STAGE_COUNT > 1
    digitalWrite(COIL_PIN_2, LOW);
#endif
    digitalWrite(CHARGE_PIN, LOW);
  }

  if (CurrentStage != 0)
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
      DischargeCapacitors();
    }
  }
  if (Complete && SensorTime1 > StartTime1)
  {
    Serial.println("Stage 1 took " + String(SensorTime1 - StartTime1) + " us");
    DebugPrint("Stage 1 took " + String(SensorTime1 - StartTime1) + " us");
    if (STAGE_COUNT > 1)
    {
      Serial.println("Stage 2 took " + String(SensorTime2 - StartTime2) + " us");
      DebugPrint("Stage 2 took " + String(SensorTime2 - StartTime2) + " us");
      Serial.println("Total time = " + String(SensorTime2 - StartTime1) + " us");
      DebugPrint("Total time = " + String(SensorTime2 - StartTime1) + " us");
    }

    Complete = false;
    DischargeCapacitors();
  }
}