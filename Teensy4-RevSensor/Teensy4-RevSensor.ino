#include <Watchdog_t4.h>

#include "RevSensor.h"

WDT_T4<WDT1> wdt;


constexpr int k_sensor1 = 2;
constexpr int k_sensor2 = 3;
constexpr int floorVal = 300;
constexpr int ceilingVal = 300;

RevSensor sensorA(Wire);
RevSensor sensorB(Wire1);

double scaleOutput(double unit) {
  return  (unit * (4095 - floorVal - ceilingVal)) + floorVal;
}

void myCallback() {
  Serial.println("WATCHDOG TIMEOUT");
}

void setup() {
  Serial.begin(9600);
  sensorA.init();
  sensorB.init();

  pinMode(k_sensor1, OUTPUT);
  pinMode(k_sensor2, OUTPUT);

  analogWriteResolution(12);

  WDT_timings_t config;
  config.trigger = 1; /* in seconds, 0->128 */
  config.timeout = 2; /* in seconds, 0->128 */
  config.callback = myCallback;
  wdt.begin(config);
  pinMode(13, OUTPUT);
}

unsigned long long lastReadTimestampA = 0;
unsigned long long lastReadTimestampB = 0;

RevSensorReading readingA;
RevSensorReading readingB;
void loop() {
  unsigned long long timestamp = millis();
  if (sensorA.get(&readingA)) {
    lastReadTimestampA = timestamp;
  }

  if (timestamp - lastReadTimestampA < 200) {
    analogWrite(k_sensor1, scaleOutput(readingA.red / static_cast<double>(readingA.red + readingA.blue)));
  } else {
    analogWrite(k_sensor1, floorVal / 2);
  }

  if (sensorB.get(&readingB)) {
    lastReadTimestampB = timestamp;
  }

  if (timestamp - lastReadTimestampB < 200) {
    analogWrite(k_sensor2, scaleOutput(readingB.red / static_cast<double>(readingB.red + readingB.blue)));
  } else {
    analogWrite(k_sensor2, floorVal / 2);
  }
  wdt.feed();
  
  digitalWrite(13, millis() % 10 < 2);
}
