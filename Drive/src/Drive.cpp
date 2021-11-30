#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#include <WiFi.h>
#include <esp_now.h>

uint8_t PeerMac[6] = {0x24, 0x6F, 0x28, 0x17, 0xC1, 0x9C};

//outputs
#define DO_MOTORS

#ifdef DO_MOTORS
// #define RIGHT_PWM_PIN 21
// #define RIGHT_FORWARD_PIN 19
// #define RIGHT_REVERSE_PIN 18
// #define LEFT_PWM_PIN 17
// #define LEFT_FORWARD_PIN 16
// #define LEFT_REVERSE_PIN 4
// #define RIGHT_PWM_PIN 18
// #define RIGHT_FORWARD_PIN 19
// #define RIGHT_REVERSE_PIN 21
// #define LEFT_PWM_PIN 4
// #define LEFT_FORWARD_PIN 16
// #define LEFT_REVERSE_PIN 17

#define RIGHT_PWM_PIN 4
#define RIGHT_FORWARD_PIN 16
#define RIGHT_REVERSE_PIN 17
#define LEFT_PWM_PIN 18
#define LEFT_FORWARD_PIN 19
#define LEFT_REVERSE_PIN 21

#define LEFT_CHANNEL 0
#define RIGHT_CHANNEL 1
#endif

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
const int MaxRPM = 15;         // Adjustable range of 28BYJ-48 stepper is 0~17 rpm
const float MaxSpeed = stepsPerRevolution * MaxRPM / 60.0f;

long mapf(long x, long in_min, long in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// initialize the stepper library on pins 8 through 11:
AccelStepper myStepper(AccelStepper::FULL4WIRE, 33, 26, 25, 27);

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
StateMsg gState;

const float MaxAccel = 512.0;
float LeftSpeed = 0;
float RightSpeed = 0;

const unsigned long LinkTimeoutUS = 2 * 1000 * 1000;
volatile unsigned long LastRecvTimeUS = micros();
volatile bool Shutdown = false;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent data!" : "Failed to send data");
}

void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len)
{
  StateMsg* msg = (StateMsg*)incomingData;
  gState = *msg;
  LastRecvTimeUS = micros();
  if (Shutdown)
  {
    Serial.println("Regained signal!\n");
    Shutdown = false;
  }

  //turret rotation
  //figure out the speed we need to go
  float panSpeed = mapf(gState.TurretTurnSpeed, -255, 255, -MaxSpeed, MaxSpeed);
  Serial.println(String(gState.TurretTurnSpeed) + " = " + String(panSpeed));
  myStepper.setSpeed(panSpeed);
}

unsigned long LastUpdateUS;
void setup() {
  // put your setup code here, to run once:

#ifdef DO_MOTORS
  ledcSetup(LEFT_CHANNEL, 5000, 8);
  ledcSetup(RIGHT_CHANNEL, 5000, 8);
  //pinMode(LEFT_PWM_PIN, OUTPUT);
  ledcAttachPin(LEFT_PWM_PIN, LEFT_CHANNEL);
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_REVERSE_PIN, OUTPUT);
  //pinMode(RIGHT_PWM_PIN, OUTPUT);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_CHANNEL);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_REVERSE_PIN, OUTPUT);

  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(LEFT_REVERSE_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_REVERSE_PIN, LOW);
#endif

  gState.LeftSpeed = 0;
  gState.RightSpeed = 0;
  gState.TurretTurnSpeed = 0;
  gState.TurretTiltSpeed = 0;
  gState.ChargePressed = 0;
  gState.TriggerPressed = 0;

  Serial.begin(115200);
  Serial.println(WiFi.macAddress());
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, PeerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  if (esp_now_register_recv_cb(OnDataRecv) != ESP_OK)
  {
    Serial.println("Failed to register receive callback\n");
    return;
  }
  Serial.println("ESP-NOW started\n");
  myStepper.stop();
  myStepper.setMaxSpeed(MaxSpeed);
  LastUpdateUS = micros();
}

void loop() {
  //see if we have lost signal
  unsigned long lastUS = LastUpdateUS;
  unsigned long nowUS = micros();
  LastUpdateUS = nowUS;
  if (!Shutdown && nowUS - LastRecvTimeUS > LinkTimeoutUS)
  {
    //lost signal!
    Serial.println("Lost RC signal");
    Shutdown = true;
    myStepper.setSpeed(0);
#ifdef DO_MOTORS
    LeftSpeed = 0;
    RightSpeed = 0;
    ledcWrite(LEFT_PWM_PIN, 0);
    ledcWrite(RIGHT_PWM_PIN, 0);
    digitalWrite(LEFT_FORWARD_PIN, LOW);
    digitalWrite(LEFT_REVERSE_PIN, LOW);
    digitalWrite(RIGHT_FORWARD_PIN, LOW);
    digitalWrite(RIGHT_REVERSE_PIN, LOW);
#endif
  }
  else if (!Shutdown)
  {
    float maxAccel = MaxAccel * (nowUS - lastUS) / 1000000.0;
    //Serial.println("elapsed = " + String(nowUS - lastUS) + " maxAccel = " + String(maxAccel));
    if (gState.LeftSpeed > LeftSpeed)
      LeftSpeed = min(LeftSpeed + maxAccel, (float)gState.LeftSpeed);
    else if (gState.LeftSpeed < LeftSpeed)
      LeftSpeed = max(LeftSpeed - maxAccel, (float)gState.LeftSpeed);
    if (gState.RightSpeed > RightSpeed)
      RightSpeed = min(RightSpeed + maxAccel, (float)gState.RightSpeed);
    else if (gState.RightSpeed < RightSpeed)
      RightSpeed = max(RightSpeed - maxAccel, (float)gState.RightSpeed);
    //Serial.println("Left speed = " + String(LeftSpeed) + "-> " + String(gState.LeftSpeed) + "; Right speed = " + String(RightSpeed) + " -> " + String(gState.RightSpeed));

#ifdef DO_MOTORS
    //left speed
    int16_t realLeftSpeed = 0;
    if (LeftSpeed > 0)
      realLeftSpeed = map(LeftSpeed, 0, 255, 50, 255);
    else if (LeftSpeed < 0)
      realLeftSpeed = map(LeftSpeed, -255, 0, -255, -50);
    int16_t realRightSpeed = 0;
    if (RightSpeed > 0)
      realRightSpeed = map(RightSpeed, 0, 255, 50, 255);
    else if (RightSpeed < 0)
      realRightSpeed = map(RightSpeed, -255, 0, -255, -50);
      
    
    if (LeftSpeed == 0)
    {
      ledcWrite(LEFT_CHANNEL, 0);
      digitalWrite(LEFT_FORWARD_PIN, HIGH);
      digitalWrite(LEFT_REVERSE_PIN, HIGH);
    }
    else if (LeftSpeed > 0)
    {
      digitalWrite(LEFT_REVERSE_PIN, LOW);
      digitalWrite(LEFT_FORWARD_PIN, HIGH);
      ledcWrite(LEFT_CHANNEL, realLeftSpeed);
    }
    else //LeftSpeed < 0
    {
      digitalWrite(LEFT_FORWARD_PIN, LOW);
      digitalWrite(LEFT_REVERSE_PIN, HIGH);
      ledcWrite(LEFT_CHANNEL, -realLeftSpeed);
    }
  
    //right speed
    //the right motor is mounted "backwards", so reverse the direction
    if (RightSpeed == 0)
    {
      ledcWrite(RIGHT_CHANNEL, 0);
      digitalWrite(RIGHT_FORWARD_PIN, HIGH);
      digitalWrite(RIGHT_REVERSE_PIN, HIGH);
    }
    else if (RightSpeed > 0)
    {
      digitalWrite(RIGHT_REVERSE_PIN, HIGH);
      digitalWrite(RIGHT_FORWARD_PIN, LOW);
      ledcWrite(RIGHT_CHANNEL, realRightSpeed);
    }
    else //gState.RightSpeed < 0
    {
      digitalWrite(RIGHT_FORWARD_PIN, HIGH);
      digitalWrite(RIGHT_REVERSE_PIN, LOW);
      ledcWrite(RIGHT_CHANNEL, -realRightSpeed);
    }
#endif
  }

  while (myStepper.runSpeed())
  {
    //Serial.println("Step!");
  }
  //else
  {
    //maybe this makes recv() work more reliably?
    yield();
    //Serial.println("NO STEP");
  }

  if (Shutdown)
  {
    delay(100);
  }
}
