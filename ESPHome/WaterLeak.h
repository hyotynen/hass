#include "esphome.h"

class WaterLeakComponent : public Component, public Sensor {
  public:

    // Wait for everything to be ready before starting the scan
    float get_setup_priority() const override { return esphome::setup_priority::LATE; }

    Sensor *deviation_sensor = new Sensor();
    Sensor *tripped_sensor = new Sensor();

    const uint32_t SEND_FREQUENCY = 60000; // ms
    unsigned long lastSendTime = 0;

    // LEAKROPE SENSOR
    const int button1 = 4;          // digital pin kalibrointinapille (4 => D2)
    const int sounder1 = 5;         // digital pin summerille (5 => D1)
    const int ropePin = 0;          // analog pin leakrope (0 => A0)

    // Buzzer
    const int warning = 10000;
    const int fault = 1000;
    const int alarm = 500;
    const int off = 0;
    long Buzzerinterval = 0;
    long BuzzerPreviousMillis = 0;

    // Button
    int buttonState = 0;
    int lastButtonState = 0;

    // Calculation
    static const int numReadings = 10;
    int readings[numReadings];
    int readIndex = 0;
    int total = 0;
    int data = 0;
    int ropedata;
    int ropebaseline;
    int ropedeviation;
    int tripped = false;

    // Automatic calibration
    const uint32_t AUTOCALIBRATION = 3600000; // every one hour
    unsigned long calibrationTime = 0;

    void setup() override {
        pinMode(A0, INPUT);
        pinMode(button1, INPUT_PULLUP);
        pinMode(sounder1, OUTPUT);
    }

    void loop() override {

    ropedata = averager();
    ropedeviation = ((float)ropedata / (float)ropebaseline) * 100;

    if (ropedeviation < 90 && ropedeviation>85)
    {
      tripped = true;
      buzzer(warning);
    }

    else if (ropedeviation < 85 && ropedeviation>0)
    {
      tripped = true;
      buzzer(alarm);
    }

    else if (ropedeviation > 110)
    {
      tripped = true;
      buzzer(fault);
    }

    else
    {
      tripped = false;
      buzzer(off);
    }

  // asetetaan anturin perustaso kun nappia painetaan
  buttonState = digitalRead(button1);
  if (buttonState != lastButtonState) {
    if (buttonState == HIGH)
    {
      setbaseline();
    }
  }
  lastButtonState = buttonState;

  // MySensors update
  if(millis() - lastSendTime >= SEND_FREQUENCY)
  {
    lastSendTime += SEND_FREQUENCY;
    tripped_sensor->publish_state(tripped);
    delay(100);
    deviation_sensor->publish_state(ropedeviation);
  }

  // automaattinen kalibrointi tunnin välein
  if (millis() - calibrationTime >= AUTOCALIBRATION)
  {
    calibrationTime += AUTOCALIBRATION;
    if ((ropedeviation < 110) && (ropedeviation > 90))
    {
      ropebaseline = ropedata;
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
void buzzer(int Buzzerinterval)
{
  unsigned long currentMillis = millis();
  if (Buzzerinterval != 0) {
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
  }
}

};
