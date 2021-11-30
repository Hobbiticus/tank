#include <Arduino.h>

#include <WiFi.h>
#include <esp_now.h>

//inputs
#define MOVE_X_PIN 34
#define MOVE_Y_PIN 39
#define PAN_PIN 33
#define TILT_PIN 32
#define CHARGE_PIN 25
#define TRIGGER_PIN 22
#define POWER_INPUT_PIN 36
#define POT_INPUT_PIN 35

//outputs
#define POWER_LIGHT_PIN 26
#define LINK_LIGHT_PIN 27
#define CHARGE_LIGHT_PIN 13

const int PWMFreq = 5000;
const int LEDResolution = 8;

const int LEDChannel = 0;
const int LEDBrightness = 256 - 40;

const int ChargeLEDChannel = 1;
const int ChargeLEDHalfBrightness = 240;
const int ChargeLEDFullBrightness = 0;

unsigned int UpdateTimeMS = 100;
unsigned int SparseUpdateFrequencyUS = 500 * 1000;

//94:b9:7e:d4:b7:19
uint8_t TurretMAC[6] = { 0x94, 0xb9, 0x7e, 0xd4, 0xb7, 0x18 };
uint8_t DriveMAC[6] = { 0xF0, 0x08, 0xD1, 0xD3, 0x6C, 0x90 };

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

bool LastSendGood[2] = {false, false};
enum LinkState
{
  LINK_DOWN = 0,
  LINK_PARTIAL,
  LINK_UP
};
LinkState LastLinkState = LINK_DOWN;
LinkState CurrLinkState = LINK_DOWN;
const uint32_t LinkBlinkTimeMS = 200;
uint32_t LastLinkBlinkTime = 0;
bool LinkLightStatus = false;

enum PowerState
{
  POWER_LOW_RC = 0,
  POWER_LOW_DRIVE,  //eventually
  POWER_LOW_TURRET, //eventually
  POWER_GOOD,
};
PowerState CurrPowerState = POWER_GOOD;
bool PowerRC = true;
bool PowerDrive = true;
bool PowerTurret = true;
//morse code (on, off, on, off, etc)
const uint32_t MORSE_DOT = 400;
const uint32_t MORSE_DASH = 800;
const uint32_t MORSE_PAUSE = 200;
const uint32_t MORSE_EOL = 1500;
const uint32_t PowerBlinkTimeRCMS[6] = {MORSE_DOT, MORSE_PAUSE, MORSE_DASH, MORSE_PAUSE, MORSE_DOT, MORSE_EOL}; //"R" .-.
const uint32_t PowerBlinkTimeTurretMS[2] = {MORSE_DASH, MORSE_EOL}; //"T" -
const uint32_t PowerBlinkTimeDriveMS[6] = {MORSE_DASH, MORSE_PAUSE, MORSE_DOT, MORSE_PAUSE, MORSE_DOT, MORSE_EOL}; //"D" -..
const uint32_t* PowerBlinkTimings[3] = { PowerBlinkTimeRCMS, PowerBlinkTimeDriveMS, PowerBlinkTimeTurretMS };
const int PowerBlinkTimingLengths[3] = {6, 2, 6};
uint32_t LastPowerBlinkTimeMS = 0;
short PowerBlinkSequence = 0;


void ChangePowerState(PowerState state)
{
  CurrPowerState = state;
  LastPowerBlinkTimeMS = millis();
  PowerBlinkSequence = 0;
  //digitalWrite(POWER_LIGHT_PIN, state == POWER_GOOD ? LOW : HIGH);
  ledcWrite(LEDChannel, state == POWER_GOOD ? LEDBrightness : 256);
  Serial.println("Power state changed to " + String(state) + " at " + String(LastPowerBlinkTimeMS));
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  char out[256] = {0};
  snprintf(out, sizeof(out), "%02hhX:%02hhX:%02hhX:%02hhX:%02hhX:%02hhX", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.println(String(out) + ": " + (status == ESP_NOW_SEND_SUCCESS ? "Sent data!" : "Failed to send data"));
  LinkState nextLinkState = LINK_DOWN;
  LastLinkState = CurrLinkState;
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    if (memcmp(mac_addr, TurretMAC, 6) == 0)
      LastSendGood[0] = true;
    else
      LastSendGood[1] = true;
    if (LastSendGood[0] && LastSendGood[1])
    {
      digitalWrite(LINK_LIGHT_PIN, LOW);
      LinkLightStatus = true;
      nextLinkState = LINK_UP;
    }
  }
  else
  {
    if (memcmp(mac_addr, TurretMAC, 6) == 0)
      LastSendGood[0] = false;
    else
      LastSendGood[1] = false;
    if (!LastSendGood[0] && !LastSendGood[1])
    {
      digitalWrite(LINK_LIGHT_PIN, HIGH);
      LinkLightStatus = false;
      nextLinkState = LINK_DOWN;
    }
  }
  if (LastSendGood[0] != LastSendGood[1])
    nextLinkState = LINK_PARTIAL;

  if (nextLinkState == LINK_PARTIAL && LastLinkState != LINK_PARTIAL)
  {
    //reverse the link light
    if (LinkLightStatus)
      digitalWrite(LINK_LIGHT_PIN, HIGH);
    else
      digitalWrite(LINK_LIGHT_PIN, LOW);
    LinkLightStatus = !LinkLightStatus;
    LastLinkBlinkTime = millis();
  }
  CurrLinkState = nextLinkState;
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  //do nothing for now
  Serial.println("Received " + String(len) + " bytes from " + String(mac[0]) + ":" + String(mac[1]) + ":" + String(mac[2]) + ":" + String(mac[3]) + ":" + String(mac[4]) + ":" + String(mac[5]));
  if (memcmp(TurretMAC, mac, 6) == 0)
  {
    StatusMsg* msg = (StatusMsg*)incomingData;
    if (msg->ChargeState == 0)
    {
      //digitalWrite(CHARGE_LIGHT_PIN, HIGH);
      ledcWrite(ChargeLEDChannel, 256);
    }
    else if (msg->ChargeState == 1)
    {
      ledcWrite(ChargeLEDChannel, ChargeLEDHalfBrightness);
    }
    else
    {
      //digitalWrite(CHARGE_LIGHT_PIN, LOW);
      ledcWrite(ChargeLEDChannel, ChargeLEDFullBrightness);
    }
  }
}




//#define X_MIN 1100
//#define X_MAX 4095
//#define X_DEAD_MIN 1900
//#define X_DEAD_MAX 2000
//#define Y_MIN 0
//#define Y_MAX 2500
//#define Y_DEAD_MIN 1500
//#define Y_DEAD_MAX 1850

#define X_MIN 0
#define X_MAX 4095
#define X_DEAD_MIN 1850
#define X_DEAD_MAX 2000
#define Y_MIN 0
#define Y_MAX 4095
#define Y_DEAD_MIN 1850
#define Y_DEAD_MAX 2000

#define PAN_MIN 0
#define PAN_MAX 4095
#define PAN_DEAD_MIN 1825
#define PAN_DEAD_MAX 1925

#define TILT_STICK_MIN 0
#define TILT_STICK_MAX 4095
#define TILT_DEAD_MIN 1825
#define TILT_DEAD_MAX 1925


//(linear is relative)
int16_t StickToLinear(int value, int stickMin, int stickMax, int deadMin, int deadMax)
{
  //turn 0-4095 into -255 to 255

  //remove dead zone
  if (value < deadMin)
  {
    int16_t ret =  map(value, stickMin, deadMin, -255, 0);
    if (ret < -255)
      return -255;
    return ret;
  }
  if (value > deadMax)
  {
    int16_t ret = map(value, deadMax, stickMax, 0, 255);
    if (ret > 255)
      return 255;
    return ret;
  }
  return 0;
}

int16_t StickToWhatever(int value, int16_t scale)
{
  //TODO: may need to calibrate dead zones
  if (value < 2100 && value > 1700)
    return 0;

  float f = value / 4096.0;
  f -= 0.5;
  return (int16_t)(f * scale);
}


//0:   full left, full right
//45:  full left, 0 right
//90:  full left, back right
//135: 0 left,    back right
//180: back left, back right
//225: back left, 0 right
//270: back left, full right
//315: 0 left,    full right

//NOTE: x is left/right, y is forward/reverse
int16_t StickToRight(int x, int y)
{
  float angle = atan2(y, x);
  if (angle < 0)
    angle += 2 * PI;
  angle *= 180 / 3.1415926535;
  float magnitude = sqrt(x * x + y * y);
  if (magnitude > 255)
    magnitude = 255;
  //Serial.println("Angle = " + String(angle) + ", magnitude = " + String(magnitude));
    
  if (angle <= 90)
    return magnitude;
  else if (angle <= 180)
    return -map(angle, 90, 180, -magnitude, magnitude);
  if (angle <= 270)
    return -magnitude;
  else
    return map(angle, 270, 360, -magnitude, magnitude);
}

int16_t StickToLeft(int x, int y)
{
  float angle = atan2(y, x);
  if (angle < 0)
    angle += 2 * PI;
  angle *= 180 / 3.1415926535;
  float magnitude = sqrt(x * x + y * y);
  if (magnitude > 255)
    magnitude = 255;

  if (angle <= 90)
    return -map(angle, 0, 90, magnitude, -magnitude);
  else if (angle <= 180)
    return magnitude;
  else if (angle <= 270)
    return -map(angle, 180, 270, -magnitude, magnitude);
  else
    return -magnitude;
}

//=============================================================================
//=============================================================================
//=============================================================================

StateMsg gLastState;
unsigned long gLastSendTime;

void setup()
{
  // put your setup code here, to run once:
  pinMode(MOVE_X_PIN, INPUT_PULLUP);
  pinMode(MOVE_Y_PIN, INPUT_PULLUP);
  pinMode(PAN_PIN, INPUT_PULLUP);
  pinMode(TILT_PIN, INPUT_PULLUP);
  pinMode(CHARGE_PIN, INPUT_PULLUP);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(POWER_INPUT_PIN, INPUT_PULLUP);

  //pinMode(POWER_LIGHT_PIN, OUTPUT);
  pinMode(LINK_LIGHT_PIN, OUTPUT);
  //pinMode(CHARGE_LIGHT_PIN, OUTPUT);
  //digitalWrite(POWER_LIGHT_PIN, LOW);
  digitalWrite(LINK_LIGHT_PIN, HIGH);
  //digitalWrite(CHARGE_LIGHT_PIN, HIGH);

  ledcSetup(LEDChannel, PWMFreq, LEDResolution);
  ledcAttachPin(POWER_LIGHT_PIN, LEDChannel);
  ledcWrite(LEDChannel, LEDBrightness);

  ledcSetup(ChargeLEDChannel, PWMFreq, LEDResolution);
  ledcAttachPin(CHARGE_LIGHT_PIN, ChargeLEDChannel);
  ledcWrite(ChargeLEDChannel, 256);

  Serial.begin(115200);
  Serial.println(WiFi.macAddress());
  WiFi.mode(WIFI_MODE_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, TurretMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add turret peer");
    return;
  }
  memcpy(peerInfo.peer_addr, DriveMAC, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add drive peer");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW started");

  gState.LeftSpeed = 0;
  gState.RightSpeed = 0;
  gState.TurretTurnSpeed = 0;
  gState.TurretTiltSpeed = 0;
  gState.ChargePressed = 0;
  gState.TriggerPressed = 0;
  gState.PowerLevel = 0;
  gLastSendTime = micros() - UpdateTimeMS * 1000;
  gLastState = gState;
}

void CheckRCPower()
{
    int reading = analogRead(POWER_INPUT_PIN);
    //TODO: power calibration?
    Serial.println("Power reading = " + String(reading));
    bool good = reading > 3000; //TODO: figure out this threshold
    if (!good && PowerRC)
    {
      //switch to blinking power
      ChangePowerState(POWER_LOW_RC);
    }
    if (good && !PowerRC)
    {
      if (PowerDrive && PowerTurret)
        ChangePowerState(POWER_GOOD);
    }
    PowerRC = good;
}

void loop()
{
  CheckRCPower();
  uint32_t nowMS = millis();

  //update LED blinkiness
  if (CurrPowerState != POWER_GOOD)
  {
    if (PowerBlinkSequence < 0)
    {
      if (nowMS - LastPowerBlinkTimeMS > MORSE_EOL)
      {
        //Serial.println("Done pause at " + String(nowMS) + " or " + String(nowMS - LastPowerBlinkTimeMS));
        //done initial pause
        //digitalWrite(POWER_LIGHT_PIN, LOW);
        ledcWrite(LEDChannel, LEDBrightness);
        PowerBlinkSequence = 0;
        //Serial.println("Changing state to " + String(PowerBlinkSequence) + " at " + String(nowMS) + " or " + String(nowMS - LastPowerBlinkTimeMS));
        LastPowerBlinkTimeMS += MORSE_EOL;
      }
    }
    else
    {
      unsigned short nextStateTimeMS = PowerBlinkTimings[CurrPowerState][PowerBlinkSequence];
      if (nowMS - LastPowerBlinkTimeMS > nextStateTimeMS)
      {
        //digitalWrite(POWER_LIGHT_PIN, PowerBlinkSequence % 2 ? HIGH : LOW);
        ledcWrite(LEDChannel, PowerBlinkSequence % 2 ? 256 : LEDBrightness);
        PowerBlinkSequence++;
        if (PowerBlinkSequence >= PowerBlinkTimingLengths[CurrPowerState])
          PowerBlinkSequence = 0;
        //Serial.println("Changing state to " + String(PowerBlinkSequence) + " at " + String(nowMS) + " or " + String(nowMS - LastPowerBlinkTimeMS));
        LastPowerBlinkTimeMS += nextStateTimeMS;
      }
    }
  }
  
  if (CurrLinkState == LINK_PARTIAL)
  {
    if (nowMS - LastLinkBlinkTime > LinkBlinkTimeMS)
    {
      //blink the light!!
      LastLinkBlinkTime += LinkBlinkTimeMS;
      if (LinkLightStatus)
        digitalWrite(LINK_LIGHT_PIN, HIGH);
      else
        digitalWrite(LINK_LIGHT_PIN, LOW);
      LinkLightStatus = !LinkLightStatus;
    }
  }


  int move_x = analogRead(MOVE_X_PIN);
  int move_y = analogRead(MOVE_Y_PIN);
  int pan = analogRead(PAN_PIN);
  int tilt = analogRead(TILT_PIN);
  int pot = analogRead(POT_INPUT_PIN);
  bool charge = digitalRead(CHARGE_PIN) == LOW;
  bool trigger = digitalRead(TRIGGER_PIN) == LOW;
  int16_t linear_x = StickToLinear(move_x, X_MIN, X_MAX, X_DEAD_MIN, X_DEAD_MAX);
  int16_t linear_y = StickToLinear(move_y, Y_MIN, Y_MAX, Y_DEAD_MIN, Y_DEAD_MAX);
  int16_t left = StickToLeft(linear_x, linear_y);
  int16_t right = StickToRight(linear_x, linear_y);
  //Serial.println("move_x = " + String(move_x) + ", move_y = " + String(move_y) + ", pan = " + String(pan) + ", tilt = " + String(tilt) + ", Charge = " + String(charge) + ", Trigger = " + String(trigger) + ", Pot = " + String(pot));
  
  ////Serial.println("x = " + String(linear_x) + ", y = " + String(linear_y));
  ////Serial.println("Left = " + String(left) + ", Right = " + String(right));
  gState.ChargePressed = charge;
  gState.TriggerPressed = trigger;

  gState.LeftSpeed = left;
  gState.RightSpeed = right;
  gState.TurretTurnSpeed = StickToLinear(pan, PAN_MIN, PAN_MAX, PAN_DEAD_MIN, PAN_DEAD_MAX);
  gState.TurretTiltSpeed = StickToLinear(tilt, TILT_STICK_MIN, TILT_STICK_MAX, TILT_DEAD_MIN, TILT_DEAD_MAX);
  gState.PowerLevel = map(pot, 0, 4095, 255, 0);
  //Serial.println("X: " + String(linear_x) + "; Y: " + String(linear_y) + "; Pan: " + String(gState.TurretTurnSpeed) + "; Tilt: " + String(gState.TurretTiltSpeed) + "; Power = " + String(gState.PowerLevel));

  //only send an update if something changed or it's been too long since the last update
  if (memcmp(&gState, &gLastState, sizeof(gState)) != 0 || micros() - gLastSendTime > SparseUpdateFrequencyUS)
  {
    esp_now_send(NULL, (uint8_t*)&gState, sizeof(gState));
    gLastSendTime = micros();
    gLastState = gState;
  }

  delay(UpdateTimeMS);
}
