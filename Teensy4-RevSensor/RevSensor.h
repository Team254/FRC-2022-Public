#pragma once
#include <stdint.h>
#include <Wire.h>

struct RevSensorReading {
  uint32_t red;
  uint32_t blue;
  uint32_t green;
  uint32_t infrared;
  uint16_t proximity;
};

using RevSensorReading = struct RevSensorReading;

class RevSensor {
  public:
    RevSensor(TwoWire& iface) : m_iface{iface} {};
    void init();
    boolean get(RevSensorReading *reading);
  private:
    size_t m_failures = 0;
    void reinit();
    void hardReset();
    TwoWire& m_iface;
};
