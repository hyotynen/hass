#include "esphome.h"

class WaterMeterComponent : public Component, public Sensor {
  public:

    // Wait for everything to be ready before starting the scan
    float get_setup_priority() const override { return esphome::setup_priority::LATE; }

    Sensor *flow_sensor = new Sensor();
    Sensor *volume_sensor = new Sensor();
    Sensor *duration_sensor = new Sensor();

    #define PULSE_FACTOR 120000
    #define MAX_FLOW 40             // Max flow (l/min) value to report. This filters outliers.
    uint32_t SEND_FREQUENCY = 30000;           // Minimum time between send (in milliseconds). We don't want to spam the gateway.
    uint32_t READ_FREQUENCY = 10;              // ms between sensor reads, too fast will make the unit unresponsive
    double ppl = ((double)PULSE_FACTOR) / 1000;      // Pulses per liter

    volatile uint32_t pulseCount = 0;
    uint32_t oldPulseCount = 0;
    volatile uint32_t lastBlink = 0;
    volatile double flow = 0;
    uint32_t newBlink = 0;
    double oldflow = 0;
    double volume = 0;
    double oldvolume = 0;
    uint32_t lastSend = 0;
    uint32_t lastPulse = 0;
    uint32_t lastRead = 0;
    double curVolume = 0;
    double oldCurVolume = 0;
    uint32_t startPulse = 0;
    uint32_t duration = 0;
    uint32_t flowStart = 0;
    uint32_t readDelay = 5000;       // delay before first read to get network ready

    // PINS
    const int ledPin = LED_BUILTIN;   // onboard LED
    int sensorPin = A0;

    // Calculation
    float distance, oldDistance, sens;
    float alpha = 0.02;               // smoothing the measurements, smaller is smoother
    bool trend = true;                // decreasing = false, increasing = true
    float slope = 0;                  // slope value
    float threshold = 0.5;            // threshold of two consecutive measurements, higher value for more distance difference required. lower value increases false positives
    float ascendLimit = threshold;    // ascend limit of oldest and newest measurement
    float descendLimit = -threshold;  // descend limit of oldest and newest measurement

    void setup() override {
      pulseCount = 0;
      lastSend = millis();
      pinMode(ledPin, OUTPUT);        // initialize onboard LED as output
    }

    void loop() override {

      uint32_t currentTime = millis();

      if ((currentTime - lastRead) > READ_FREQUENCY && currentTime > readDelay) {
        sens = analogRead(sensorPin);
        if (sens < 1024 && sens > 0) // Eliminite error values beyond sensor range
        {
          lastRead = currentTime;
          readDelay = 0;
          distance = (1 - alpha) * distance + alpha * sens;

          slope = distance - oldDistance;
          oldDistance = distance;

          // start of a slope
          if (slope > ascendLimit && trend == false)
          {
            trend = true;
            analogWrite(ledPin, 0);
            delay(10);
            analogWrite(ledPin, 1023);
            onPulse();
          }

          // end of a slope, peak reached
          if (slope < descendLimit && trend == true)
          {
            trend = false;
            analogWrite(ledPin, 0);
            delay(10);
            analogWrite(ledPin, 1023);
            onPulse();
          }
        }
      }

      // Only send values at a maximum frequency or woken up from sleep
      if (currentTime - lastSend > SEND_FREQUENCY) {
        lastSend = currentTime;

        // No Pulse count increase in 30 sec
        if (currentTime - lastPulse > 30000) {
          Serial.println("Flow stopped");
          flow = 0;
          startPulse = pulseCount;
          flowStart = lastPulse;
        }

        if (flow != oldflow) {
          oldflow = flow;

          Serial.print("l/min: ");
          Serial.println(flow);

          // Check that we don't get unreasonable large flow value.
          // could happen when long wraps or false interrupt triggered
          if (flow < ((uint32_t)MAX_FLOW)) {
              flow_sensor->publish_state(flow);
          }
        }

        // Pulse count has changed
        if (pulseCount != oldPulseCount) {
          oldPulseCount = pulseCount;

          Serial.print("pulsecount: ");
          Serial.println(pulseCount);

          double volume = ((double)pulseCount / ((double)PULSE_FACTOR));
          if (volume != oldvolume) {
            oldvolume = volume;
            Serial.print("volume: ");
            Serial.println(volume, 4);
            volume_sensor->publish_state(volume*1000);
          }
        }

        curVolume = (((double)pulseCount - (double)startPulse) / ((double)PULSE_FACTOR));
        duration = (lastPulse - flowStart) / 1000;
        if (curVolume != oldCurVolume) {
          oldCurVolume = curVolume;
          Serial.print("Current flow volume: ");
          Serial.print(curVolume, 4);
          Serial.print(" in ");
          Serial.print(duration);
          Serial.println(" seconds");
          duration_sensor->publish_state(duration);
        }
      }
    }

    void onPulse()
    {
      uint32_t newBlink = micros();
      uint32_t interval = newBlink - lastBlink;

      if (interval != 0) {
        if (flow == 0) {
          Serial.println("Flow started");
          startPulse = pulseCount;
          flowStart = millis();
        }
        lastPulse = millis();
        flow = (60000000.0 / interval) / ppl;
      }
      lastBlink = newBlink;
      pulseCount++;
    }
};
