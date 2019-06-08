// Leakrope -sensori, MySensors & Home Assistant
// Kimmo Hyötynen, https://hyotynen.iki.fi/kotiautomaatio

// MYSENSOR
#define MY_DEBUG
#define MY_GATEWAY_MQTT_CLIENT
#define MY_GATEWAY_ESP8266

// Set this node's subscribe and publish topic prefix
#define MY_MQTT_PUBLISH_TOPIC_PREFIX "mygateway1-out"
#define MY_MQTT_SUBSCRIBE_TOPIC_PREFIX "mygateway1-in"

// Set MQTT client id
#define MY_MQTT_CLIENT_ID "mysensors-1"

// Enable these if your MQTT broker requires username/password
#define MY_MQTT_USER "homeassistantuser"
#define MY_MQTT_PASSWORD "password"

// Set WIFI SSID and password
#define MY_WIFI_SSID "SSID"
#define MY_WIFI_PASSWORD "WIFIPassword"

// MQTT broker ip address.
#define MY_CONTROLLER_IP_ADDRESS 1, 1, 1, 1

// The MQTT broker port to to open
#define MY_PORT PORT

#include <ESP8266WiFi.h>
#include <MySensors.h>

#define CHILD_ID 2              // Id of the sensor child
MyMessage tripMsg(CHILD_ID, V_TRIPPED);
MyMessage deviationMsg(CHILD_ID, V_VAR3);
const uint32_t SEND_FREQUENCY = 60000; // ms
unsigned long lastSendTime = 0;

// LEAKROPE SENSOR
const int button1 = 4;          // digital pin kalibrointinapille (4 => D2)
const int sounder1 = 5;         // digital pin summerille (5 => D1)
const int ropePin = 0;          // analog pin leakrope (0 => A0)
const int serialmonitor = true; // debug

// summerin parametrit
const int warning = 10000;
const int fault = 1000;
const int alarm = 500;
const int off = 0;
long Buzzerinterval = 0;
long BuzzerPreviousMillis = 0;

// napin parametrit
int buttonState = 0;
int lastButtonState = 0;

// laskennan parametrit
const int numReadings = 10;
int readings[numReadings];
int readIndex = 0;
int total = 0;
int data = 0;
int ropedata;
int ropebaseline;
int ropedeviation;
int tripped = false;

// Automaattinen kalibrointi
const uint32_t AUTOCALIBRATION = 3600000; // every one hour
unsigned long calibrationTime = 0;

// MySensors presentation
void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Leakrope sensor", "1.0");

  // Register this device as power sensor
  present(CHILD_ID, S_WATER_LEAK);
}

void setup() {
  if (serialmonitor == true) {
    Serial.begin(9600);
    Serial.println("Serial debugging on");
  }
  
  pinMode(A0, INPUT);
  pinMode(button1, INPUT_PULLUP);
  pinMode(sounder1, OUTPUT);
}

void loop() {
  ropedata = averager();
  ropedeviation = ((float)ropedata / (float)ropebaseline) * 100;

  if (serialmonitor == true) {
    Serial.print(" Value: ");
    Serial.print(ropedata);
    Serial.print(" Baseline: ");
    Serial.print(ropebaseline);
    Serial.print(" Deviation %: ");
    Serial.println(ropedeviation);
  }

    if (ropedeviation < 90 && ropedeviation>85)
    {
      if (serialmonitor == true) {
        Serial.println("Warning!");
      }
      tripped = true;
      buzzer(warning);
    }

    else if (ropedeviation < 85 && ropedeviation>0)
    {
      if (serialmonitor == true) {
        Serial.println("Alarm!");
      }
      tripped = true;
      buzzer(alarm);
    }

    else if (ropedeviation > 110)
    {
      if (serialmonitor == true) {
        Serial.println("Fault!");
      }
      tripped = true;
      buzzer(fault);
    }

    else
    {
      if (serialmonitor == true) {
        Serial.println("Normal");
      }
      tripped = false;
      buzzer(off);
    }
  
  // asetetaan anturin perustaso kun nappia painetaan
  buttonState = digitalRead(button1);
  if (buttonState != lastButtonState) {
    if (buttonState == HIGH)
    {
      if (serialmonitor == true) {
        Serial.println("Setting baseline");
      }
      setbaseline();
    }
  }
  lastButtonState = buttonState;

  // MySensors update
  if(millis() - lastSendTime >= SEND_FREQUENCY)
  {
    lastSendTime += SEND_FREQUENCY;
    if (serialmonitor == true) {
      Serial.println("Sending values to MQTT");
    }
    send(tripMsg.set(tripped));
    delay(100);
    send(deviationMsg.set(ropedeviation));
  }

  // automaattinen kalibrointi tunnin välein
  if (millis() - calibrationTime >= AUTOCALIBRATION)
  {
    calibrationTime += AUTOCALIBRATION;
    if ((ropedeviation < 110) && (ropedeviation > 90))
    {
      ropebaseline = ropedata;
      if (serialmonitor == true) { Serial.println("Automatic calibration"); }
    }
  }

  delay(500);
}

// anturin lukemien keskiarvon laskenta
int averager()
{
  total = 0;
  for (int readindex = 0; readindex < numReadings; readindex++)
  {
    data = analogRead(ropePin);
    readings[readIndex] = data;
    total = total + readings[readIndex];
    delay(20);
  }
  return (total / numReadings);
}

// anturin perustason asetus
void setbaseline()
{
  ropebaseline = ropedata;
  if (serialmonitor == true) {
    Serial.print("Update Rope Baseline with value: ");
    Serial.println(ropebaseline);
  }

  // merkkiääni perustason asettamisesta
  digitalWrite(sounder1, HIGH);
  delay(100);
  digitalWrite(sounder1, LOW);
  delay(100);
  digitalWrite(sounder1, HIGH);
  delay(100);
  digitalWrite(sounder1, LOW);
}

// summeri
int buzzer(int Buzzerinterval)
{
  unsigned long currentMillis = millis();
  if (Buzzerinterval != 0) {
    if (serialmonitor == true) { Serial.println("Buzzer Loop"); }
    if (currentMillis - BuzzerPreviousMillis > 600) {
      digitalWrite(sounder1, LOW);
    }
    if (currentMillis - BuzzerPreviousMillis > Buzzerinterval) {
      BuzzerPreviousMillis = currentMillis;
      digitalWrite(sounder1, HIGH);
    }
  }
  else if (digitalRead(sounder1))
  {
    digitalWrite(sounder1, LOW);
    if (serialmonitor == true) { Serial.println("Turning Buzzer off"); }
  }
}
