#include "RevSensor.h"

// implementation of rev sensor adapted from code from Thad House
// https://github.com/ThadHouse/picocolorsensor

/////////////////////

enum class GainFactor
{
  k1x = 0,
  k3x = 1,
  k6x = 2,
  k9x = 3,
  k18x = 4
};

enum class LEDPulseFrequency
{
  k60kHz = 0x18,
  k70kHz = 0x40,
  k80kHz = 0x28,
  k90kHz = 0x30,
  k100kHz = 0x38,
};

enum class LEDCurrent
{
  kPulse2mA = 0,
  kPulse5mA = 1,
  kPulse10mA = 2,
  kPulse25mA = 3,
  kPulse50mA = 4,
  kPulse75mA = 5,
  kPulse100mA = 6,
  kPulse125mA = 7,
};

enum class ProximityResolution
{
  k8bit = 0x00,
  k9bit = 0x08,
  k10bit = 0x10,
  k11bit = 0x18,
};

enum class ProximityMeasurementRate
{
  k6ms = 1,
  k12ms = 2,
  k25ms = 3,
  k50ms = 4,
  k100ms = 5,
  k200ms = 6,
  k400ms = 7,
};

enum class ColorResolution
{
  k20bit = 0x00,
  k19bit = 0x10,
  k18bit = 0x20,
  k17bit = 0x30,
  k16bit = 0x40,
  k13bit = 0x50,
};

enum class ColorMeasurementRate
{
  k25ms = 0,
  k50ms = 1,
  k100ms = 2,
  k200ms = 3,
  k500ms = 4,
  k1000ms = 5,
  k2000ms = 7,
};

enum class Register
{
  kMainCtrl = 0x00,
  kProximitySensorLED = 0x01,
  kProximitySensorPulses = 0x02,
  kProximitySensorRate = 0x03,
  kLightSensorMeasurementRate = 0x04,
  kLightSensorGain = 0x05,
  kPartID = 0x06,
  kMainStatus = 0x07,
  kProximityData = 0x08,
  kDataInfrared = 0x0A,
  kDataGreen = 0x0D,
  kDataBlue = 0x10,
  kDataRed = 0x13
};

enum class MainCtrlFields
{
  kProximitySensorEnable = 0x01,
  kLightSensorEnable = 0x02,
  kRGBMode = 0x04
};

static constexpr int kAddress = 0x52;

#define INTIAL_POWERUP_STATE_FLAG 0x20

////////////////////


void RevSensor::reinit() {
  uint8_t i2cBuffer[2];
  i2cBuffer[0] = static_cast<uint8_t>(Register::kMainCtrl);
  i2cBuffer[1] = static_cast<uint8_t>(MainCtrlFields::kRGBMode) |
                 static_cast<uint8_t>(MainCtrlFields::kLightSensorEnable) |
                 static_cast<uint8_t>(MainCtrlFields::kProximitySensorEnable);
  m_iface.beginTransmission(kAddress);
  m_iface.write(i2cBuffer, 2);

  i2cBuffer[0] = static_cast<uint8_t>(Register::kProximitySensorRate);
  i2cBuffer[1] = static_cast<uint8_t>(ProximityResolution::k11bit) | static_cast<uint8_t>(ProximityMeasurementRate::k100ms);
  m_iface.write(i2cBuffer, 2);

  i2cBuffer[0] = static_cast<uint8_t>(Register::kProximitySensorPulses);
  i2cBuffer[1] = 32;
  m_iface.write(i2cBuffer, 2);
  m_iface.endTransmission();
}


void RevSensor::init() {
  m_iface.begin();
  reinit();
}

void RevSensor::hardReset() {
  uint8_t i2cBuffer[1];
  m_iface.begin();
  m_iface.beginTransmission(kAddress);
  i2cBuffer[0] = static_cast<uint8_t>(0);
  m_iface.write(i2cBuffer, 1);
  m_iface.endTransmission();
  m_iface.requestFrom(kAddress, 20);
  while (m_iface.available()) {
    m_iface.read();
  }
}

boolean RevSensor::get(RevSensorReading *reading) {
  constexpr size_t buffsize = 15;
  uint8_t i2cBuffer[buffsize];
  for (unsigned int i = 0; i < buffsize; ++i) {
    i2cBuffer[i] = 0;
  }

  unsigned int values[10];

  // Request main status + following registers
  m_iface.beginTransmission(kAddress);
  i2cBuffer[0] = static_cast<uint8_t>(Register::kMainStatus);
  m_iface.write(i2cBuffer, 1);
  m_iface.endTransmission();
  i2cBuffer[0] = 0;
  // Issue the pulses to get the data
  m_iface.requestFrom(kAddress, buffsize);

  size_t i = 0;
  while (m_iface.available() || i >= buffsize)   // device may send less than requested, also protect against more.
  {
    char c = m_iface.read();
    //Serial.print(c, HEX);
    i2cBuffer[i++] = c;
  }
  /*
  Serial.print(i);
  Serial.print(",");
  Serial.print(i2cBuffer[0]);
  Serial.println();
  */
  if (i == 0 || (i2cBuffer[0] & INTIAL_POWERUP_STATE_FLAG) != 0)
  {
    if (m_failures++ > 10) {
      hardReset(); // clears faults on i2c bus
    }
    reinit();
  } else {
    values[4] = ((i2cBuffer[1] & 0xFF) | ((i2cBuffer[2] & 0xFF) << 8)) & 0x7FF;
    values[3] = ((i2cBuffer[3] & 0xFF) | ((i2cBuffer[4] & 0xFF) << 8) | ((i2cBuffer[5] & 0xFF) << 16)) & 0x03FFFF;
    values[2] = ((i2cBuffer[6] & 0xFF) | ((i2cBuffer[7] & 0xFF) << 8) | ((i2cBuffer[8] & 0xFF) << 16)) & 0x03FFFF;
    values[1] = ((i2cBuffer[9] & 0xFF) | ((i2cBuffer[10] & 0xFF) << 8) | ((i2cBuffer[11] & 0xFF) << 16)) & 0x03FFFF;
    values[0] = ((i2cBuffer[12] & 0xFF) | ((i2cBuffer[13] & 0xFF) << 8) | ((i2cBuffer[14] & 0xFF) << 16)) & 0x03FFFF;

    if (reading == 0) return false; // null ptr
    reading->red = values[0];
    reading->blue = values[1];
    reading->green = values[2];
    reading->infrared = values[3];
    reading->proximity = values[4];
    if (i2cBuffer[0] == 0) return false; // no status reported at all
    return true;
  }
  return false;
}
