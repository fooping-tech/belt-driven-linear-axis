#include "As5600Sensor.h"

#include <Wire.h>

namespace {
constexpr uint8_t kAs5600Address = 0x36;
constexpr uint8_t kStatusRegister = 0x0B;
constexpr uint8_t kRawAngleRegister = 0x0C;
constexpr uint8_t kAngleRegister = 0x0E;
constexpr uint8_t kAgcRegister = 0x1A;
constexpr uint8_t kMagnitudeRegister = 0x1B;
constexpr uint8_t kBitBangDelayUs = 5;
constexpr float kCountsPerRev = 4096.0F;
}

As5600Sensor::As5600Sensor(uint8_t sdaPin, uint8_t sclPin) : sdaPin_(sdaPin), sclPin_(sclPin) {}

void As5600Sensor::begin() {
  Wire.begin(sdaPin_, sclPin_);
  Wire.setClock(100000);
}

bool As5600Sensor::readRegister(uint8_t reg, uint8_t* data, size_t length) {
  Wire.beginTransmission(kAs5600Address);
  Wire.write(reg);
  if (Wire.endTransmission(true) != 0) {
    return false;
  }

  delayMicroseconds(50);
  const uint8_t requested = static_cast<uint8_t>(length);
  if (Wire.requestFrom(kAs5600Address, requested) != requested) {
    return false;
  }

  for (size_t index = 0; index < length; ++index) {
    if (!Wire.available()) {
      return false;
    }
    data[index] = Wire.read();
  }
  return true;
}

bool As5600Sensor::readWord(uint8_t reg, uint16_t& value) {
  uint8_t data[2] = {0, 0};
  if (!readRegister(reg, data, sizeof(data))) {
    return false;
  }
  value = (static_cast<uint16_t>(data[0] & 0x0F) << 8) | data[1];
  return true;
}

As5600Sensor::Reading As5600Sensor::read() {
  Reading reading;
  reading.statusOk = readRegister(kStatusRegister, &reading.status, 1);
  reading.rawOk = readWord(kRawAngleRegister, reading.rawAngle);
  reading.angleOk = readWord(kAngleRegister, reading.angle);
  reading.agcOk = readRegister(kAgcRegister, &reading.agc, 1);
  reading.magnitudeOk = readWord(kMagnitudeRegister, reading.magnitude);
  reading.ok = reading.statusOk && reading.rawOk && reading.angleOk && reading.agcOk && reading.magnitudeOk;
  reading.rawDegrees = reading.rawOk ? countsToDegrees(reading.rawAngle) : 0.0F;
  reading.angleDegrees = reading.angleOk ? countsToDegrees(reading.angle) : 0.0F;
  return reading;
}

As5600Sensor::BitBangReading As5600Sensor::readBitBang() {
  Wire.end();
  bitBangRelease(sclPin_);
  bitBangRelease(sdaPin_);
  delay(20);

  BitBangReading reading;
  reading.statusOk = bitBangReadRegister(kStatusRegister, &reading.status, 1, reading.statusAckMask);

  uint8_t rawData[2] = {0, 0};
  reading.rawOk = bitBangReadRegister(kRawAngleRegister, rawData, sizeof(rawData), reading.rawAckMask);
  reading.rawAngle = (static_cast<uint16_t>(rawData[0] & 0x0F) << 8) | rawData[1];
  reading.rawDegrees = reading.rawOk ? countsToDegrees(reading.rawAngle) : 0.0F;
  reading.ok = reading.statusOk && reading.rawOk;
  begin();
  return reading;
}

uint8_t As5600Sensor::scanWire(uint8_t* addresses, size_t maxAddresses) {
  Wire.end();
  begin();
  delay(20);

  uint8_t count = 0;
  for (uint8_t address = 1; address < 0x7F; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      if (count < maxAddresses) {
        addresses[count] = address;
      }
      ++count;
    }
    delay(2);
  }
  return count;
}

uint8_t As5600Sensor::scanBitBang(uint8_t* addresses, size_t maxAddresses) {
  Wire.end();
  bitBangRelease(sclPin_);
  bitBangRelease(sdaPin_);
  delay(20);

  uint8_t count = 0;
  for (uint8_t address = 1; address < 0x7F; ++address) {
    if (bitBangProbeAddress(address)) {
      if (count < maxAddresses) {
        addresses[count] = address;
      }
      ++count;
    }
    delay(2);
  }
  begin();
  return count;
}

As5600Sensor::PinPulseResult As5600Sensor::runPinPulseTest() {
  Wire.end();
  pinMode(sclPin_, INPUT);
  pinMode(sdaPin_, INPUT);
  delay(20);

  PinPulseResult result;
  result.sclInitial = digitalRead(sclPin_);
  result.sdaInitial = digitalRead(sdaPin_);

  pinMode(sclPin_, OUTPUT);
  digitalWrite(sclPin_, LOW);
  delay(200);
  result.sclLowRead = digitalRead(sclPin_);
  result.sdaWhileSclLowRead = digitalRead(sdaPin_);
  pinMode(sclPin_, INPUT);
  delay(200);
  result.sclReleasedRead = digitalRead(sclPin_);
  result.sdaAfterSclReleaseRead = digitalRead(sdaPin_);

  pinMode(sdaPin_, OUTPUT);
  digitalWrite(sdaPin_, LOW);
  delay(200);
  result.sclWhileSdaLowRead = digitalRead(sclPin_);
  result.sdaLowRead = digitalRead(sdaPin_);
  pinMode(sdaPin_, INPUT);
  delay(200);
  result.sclFinal = digitalRead(sclPin_);
  result.sdaFinal = digitalRead(sdaPin_);
  begin();
  return result;
}

uint8_t As5600Sensor::sdaPin() const {
  return sdaPin_;
}

uint8_t As5600Sensor::sclPin() const {
  return sclPin_;
}

uint8_t As5600Sensor::address() const {
  return kAs5600Address;
}

bool As5600Sensor::magnetDetected(uint8_t status) {
  return (status & 0x20U) != 0;
}

bool As5600Sensor::magnetTooWeak(uint8_t status) {
  return (status & 0x10U) != 0;
}

bool As5600Sensor::magnetTooStrong(uint8_t status) {
  return (status & 0x08U) != 0;
}

float As5600Sensor::countsToDegrees(uint16_t counts) {
  return static_cast<float>(counts) * 360.0F / kCountsPerRev;
}

void As5600Sensor::bitBangRelease(uint8_t pin) {
  pinMode(pin, INPUT);
}

void As5600Sensor::bitBangLow(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void As5600Sensor::bitBangDelay() {
  delayMicroseconds(kBitBangDelayUs);
}

void As5600Sensor::bitBangClockHigh() {
  bitBangRelease(sclPin_);
  bitBangDelay();
}

void As5600Sensor::bitBangClockLow() {
  bitBangLow(sclPin_);
  bitBangDelay();
}

void As5600Sensor::bitBangStart() {
  bitBangRelease(sdaPin_);
  bitBangRelease(sclPin_);
  bitBangDelay();
  bitBangLow(sdaPin_);
  bitBangDelay();
  bitBangLow(sclPin_);
  bitBangDelay();
}

void As5600Sensor::bitBangStop() {
  bitBangLow(sdaPin_);
  bitBangDelay();
  bitBangRelease(sclPin_);
  bitBangDelay();
  bitBangRelease(sdaPin_);
  bitBangDelay();
}

bool As5600Sensor::bitBangWriteByte(uint8_t value) {
  for (uint8_t bit = 0; bit < 8; ++bit) {
    if ((value & 0x80U) != 0) {
      bitBangRelease(sdaPin_);
    } else {
      bitBangLow(sdaPin_);
    }
    bitBangDelay();
    bitBangClockHigh();
    bitBangClockLow();
    value <<= 1;
  }

  bitBangRelease(sdaPin_);
  bitBangDelay();
  bitBangClockHigh();
  const bool ack = digitalRead(sdaPin_) == LOW;
  bitBangClockLow();
  return ack;
}

uint8_t As5600Sensor::bitBangReadByte(bool ack) {
  uint8_t value = 0;
  bitBangRelease(sdaPin_);
  for (uint8_t bit = 0; bit < 8; ++bit) {
    value <<= 1;
    bitBangClockHigh();
    if (digitalRead(sdaPin_) == HIGH) {
      value |= 1U;
    }
    bitBangClockLow();
  }

  if (ack) {
    bitBangLow(sdaPin_);
  } else {
    bitBangRelease(sdaPin_);
  }
  bitBangDelay();
  bitBangClockHigh();
  bitBangClockLow();
  bitBangRelease(sdaPin_);
  return value;
}

bool As5600Sensor::bitBangReadRegister(uint8_t reg, uint8_t* data, size_t length, uint8_t& ackMask) {
  ackMask = 0;
  bitBangStart();
  const bool addrWriteAck = bitBangWriteByte((kAs5600Address << 1) | 0U);
  if (addrWriteAck) {
    ackMask |= 0x01U;
  }
  const bool regAck = bitBangWriteByte(reg);
  if (regAck) {
    ackMask |= 0x02U;
  }
  bitBangStart();
  const bool addrReadAck = bitBangWriteByte((kAs5600Address << 1) | 1U);
  if (addrReadAck) {
    ackMask |= 0x04U;
  }

  for (size_t index = 0; index < length; ++index) {
    data[index] = bitBangReadByte(index + 1U < length);
  }
  bitBangStop();
  return addrWriteAck && regAck && addrReadAck;
}

bool As5600Sensor::bitBangProbeAddress(uint8_t address) {
  bitBangStart();
  const bool ack = bitBangWriteByte((address << 1) | 0U);
  bitBangStop();
  return ack;
}
