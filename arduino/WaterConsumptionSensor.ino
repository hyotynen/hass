// Water consumption sensor, MySensors & Home Assistant
// Kimmo Hyötynen, https://hyotynen.iki.fi/kotiautomaatio

// MYSENSORS
// Enable debug prints to serial monitor
#define MY_DEBUG
#define MY_GATEWAY_ESP8266
#define MY_WIFI_SSID "WIFI_SSID"
#define MY_WIFI_PASSWORD "WIFI_PASSWORD"
#define MY_HOSTNAME "WaterConsumption"

// MQTT
#define MY_GATEWAY_MQTT_CLIENT
#define MY_MQTT_PUBLISH_TOPIC_PREFIX "mygateway1-out"
#define MY_MQTT_SUBSCRIBE_TOPIC_PREFIX "mygateway1-in"
#define MY_MQTT_CLIENT_ID "mysensors-1"
#define MY_MQTT_USER "USERNAME"
#define MY_MQTT_PASSWORD "PASSWORD"
#define MY_CONTROLLER_IP_ADDRESS 192, 168, X, Y
#define MY_PORT PORT_NUMBER

#include <MySensors.h>

#define CHILD_ID 3              // Id of the sensor child
#define PULSE_FACTOR 120000     // Number of blinks per m3 of your meter
#define SLEEP_MODE false        // flowvalue can only be reported when sleep mode is false.
#define MAX_FLOW 40             // Max flow (l/min) value to report. This filters outliers.

uint32_t SEND_FREQUENCY = 30000;           // Minimum time between send (in milliseconds). We don't want to spam the gateway.
uint32_t READ_FREQUENCY = 1;               // ms between sensor reads

MyMessage flowMsg(CHILD_ID, V_FLOW);
MyMessage volumeMsg(CHILD_ID, V_VOLUME);
MyMessage lastCounterMsg(CHILD_ID, V_VAR1);

double ppl = ((double)PULSE_FACTOR) / 1000;      // Pulses per liter

volatile uint32_t pulseCount = 0;
volatile uint32_t lastBlink = 0;
volatile double flow = 0;
bool pcReceived = true;
uint32_t oldPulseCount = 0;
uint32_t newBlink = 0;
double oldflow = 0;
double volume = 0;
double oldvolume = 0;
uint32_t lastSend = 0;
uint32_t lastPulse = 0;
uint32_t lastRead = 0;
uint32_t readDelay = 10000;       // delay before first read to get network ready

// LED
const int ledPin = BUILTIN_LED;   // the onboard LED
int brightness = 0;               // how bright the LED is (0 = full, 1023 = off)

// Photodiode
int sensorPin = A0;
int sensorValue = 0;

// Calculation
#include <QueueArray.h>
float distance, sens;
float alpha = 0.02;               // smoothing the measurements, smaller is smoother
bool trend = false;               // decreasing = false, increasing = true
float slope = 0;                  // slope value
float threshold = 0.5;            // threshold of two consecutive measurements, higher value for more distance difference required. lower value increases false positives
int measurementCount = 1;         // number of measurements to store. higher number for extra smoothing
QueueArray <float> measurements;  // queue of measurements
float ascendLimit = measurementCount * threshold;                // ascend limit of oldest and newest measurement
float descendLimit = -measurementCount * threshold;              // descend limit of oldest and newest measurement

bool debug = false;               // output debugging info to console

void setup()
{
  pulseCount = oldPulseCount = 0;

  // Fetch last known pulse count value from gw
  request(CHILD_ID, V_VAR1);

  lastSend = lastPulse = lastRead =  millis();

  pinMode(ledPin, OUTPUT);        // initialize onboard LED as output
  Serial.begin(9600);
}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Water Meter", "1.1");

  // Register this device as power sensor
  present(CHILD_ID, S_WATER);
}

void loop()
{
  // MySensors update
  uint32_t currentTime = millis();

  if ((currentTime - lastRead) > READ_FREQUENCY && currentTime > readDelay) {
    sens = analogRead(sensorPin);
    if (sens < 1024 && sens > 0) // Eliminite error values beyond sensor range
    {
      lastRead = currentTime;
      readDelay = 0;
      distance = (1 - alpha) * distance + alpha * sens;
  
      // pushing and popping the measurement values to the queue
      measurements.push (distance);
      if (measurements.count() > measurementCount)
      {
        slope = distance - measurements.pop();
      }
  
      // start of a slope
      if (slope > ascendLimit && trend == false)
      {
        trend = true;
        if (debug)
        {
          Serial.println("up");
          Serial.print("sensor: ");
          Serial.println(sens);
        }
        analogWrite(ledPin, 0);
        delay(10);
        analogWrite(ledPin, 1023);
        onPulse();
      }
  
      // end of a slope, peak reached
      if (slope < descendLimit && trend == true)
      {
        trend = false;
        if (debug)
        {
          Serial.println("down");
          Serial.print("sensor: ");
          Serial.println(sens);
        }
        analogWrite(ledPin, 0);
        delay(10);
        analogWrite(ledPin, 1023);
        onPulse();
      }
    }
  }

  // Only send values at a maximum frequency or woken up from sleep
  if (SLEEP_MODE || (currentTime - lastSend > SEND_FREQUENCY)) {
    lastSend = currentTime;

    if (!pcReceived) {
      //Last Pulsecount not yet received from controller, request it again
      request(CHILD_ID, V_VAR1);
      return;
    }

    // No Pulse count received in 30 sec
    if (currentTime - lastPulse > 30000) {
      flow = 0;
    }

    if (!SLEEP_MODE && flow != oldflow) {
      oldflow = flow;

      Serial.print("l/min:");
      Serial.println(flow);

      // Check that we don't get unreasonable large flow value.
      // could happen when long wraps or false interrupt triggered
      if (flow < ((uint32_t)MAX_FLOW)) {
        send(flowMsg.set(flow, 3));                   // Send flow value to gw
      }
    }

    // Pulse count has changed
    if ((pulseCount != oldPulseCount) || (!SLEEP_MODE)) {
      oldPulseCount = pulseCount;

      Serial.print("pulsecount:");
      Serial.println(pulseCount);

      send(lastCounterMsg.set(pulseCount));           // Send  pulsecount value to gw in VAR1

      double volume = ((double)pulseCount / ((double)PULSE_FACTOR));
      if ((volume != oldvolume) || (!SLEEP_MODE)) {
        oldvolume = volume;

        Serial.print("volume:");
        Serial.println(volume, 4);

        send(volumeMsg.set(volume, 4));               // Send volume value to gw
      }
    }
  }
  if (SLEEP_MODE) {
    sleep(SEND_FREQUENCY);
  }

  if (debug)
  {
//    Serial.println(sens);         // Raw sensor value
//    Serial.println(distance);     // Smoothed value
//    Serial.println(slope);
//    Serial.println(trend);
  }
}

void receive(const MyMessage &message)
{
  if (message.type == V_VAR1) {
    uint32_t gwPulseCount = message.getULong();
    pulseCount += gwPulseCount;
    flow = oldflow = 0;
    Serial.print("Received last pulse count from gw:");
    Serial.println(pulseCount);
    pcReceived = true;
  }
}

void onPulse()
{
  if (!SLEEP_MODE) {
    uint32_t newBlink = micros();
    uint32_t interval = newBlink - lastBlink;

    if (interval != 0) {
      lastPulse = millis();
      flow = (60000000.0 / interval) / ppl;
    }
    lastBlink = newBlink;
  }
  pulseCount++;
}
