#pragma once

#include <Arduino.h>

class As5600Sensor {
 public:
  struct Reading {
    bool ok = false;
    bool statusOk = false;
    bool rawOk = false;
    bool angleOk = false;
    bool agcOk = false;
    bool magnitudeOk = false;
    uint8_t status = 0;
    uint8_t agc = 0;
    uint16_t rawAngle = 0;
    uint16_t angle = 0;
    uint16_t magnitude = 0;
    float rawDegrees = 0.0F;
    float angleDegrees = 0.0F;
  };

  struct BitBangReading {
    bool ok = false;
    bool statusOk = false;
    bool rawOk = false;
    uint8_t statusAckMask = 0;
    uint8_t rawAckMask = 0;
    uint8_t status = 0;
    uint16_t rawAngle = 0;
    float rawDegrees = 0.0F;
  };

  struct PinPulseResult {
    uint8_t sclInitial = 0;
    uint8_t sdaInitial = 0;
    uint8_t sclLowRead = 0;
    uint8_t sdaWhileSclLowRead = 0;
    uint8_t sclReleasedRead = 0;
    uint8_t sdaAfterSclReleaseRead = 0;
    uint8_t sclWhileSdaLowRead = 0;
    uint8_t sdaLowRead = 0;
    uint8_t sclFinal = 0;
    uint8_t sdaFinal = 0;
  };

  As5600Sensor(uint8_t sdaPin, uint8_t sclPin);

  void begin();
  bool readRegister(uint8_t reg, uint8_t* data, size_t length);
  bool readWord(uint8_t reg, uint16_t& value);
  Reading read();
  BitBangReading readBitBang();
  uint8_t scanWire(uint8_t* addresses, size_t maxAddresses);
  uint8_t scanBitBang(uint8_t* addresses, size_t maxAddresses);
  PinPulseResult runPinPulseTest();

  uint8_t sdaPin() const;
  uint8_t sclPin() const;
  uint8_t address() const;

  static bool magnetDetected(uint8_t status);
  static bool magnetTooWeak(uint8_t status);
  static bool magnetTooStrong(uint8_t status);
  static float countsToDegrees(uint16_t counts);

 private:
  void bitBangRelease(uint8_t pin);
  void bitBangLow(uint8_t pin);
  void bitBangDelay();
  void bitBangClockHigh();
  void bitBangClockLow();
  void bitBangStart();
  void bitBangStop();
  bool bitBangWriteByte(uint8_t value);
  uint8_t bitBangReadByte(bool ack);
  bool bitBangReadRegister(uint8_t reg, uint8_t* data, size_t length, uint8_t& ackMask);
  bool bitBangProbeAddress(uint8_t address);

  uint8_t sdaPin_;
  uint8_t sclPin_;
};
