#pragma once

#include <Arduino.h>

#include "As5600Sensor.h"
#include "Axis.h"
#include "generated/BeltCartpoleConfig.h"

class CartPoleBalanceController {
 public:
  enum class Mode : uint8_t {
    Swingup,
    Lqr,
  };

  enum class StopReason : uint8_t {
    None,
    NotHomed,
    SensorFault,
    MotorFault,
    SoftLimit,
    Stall,
    UserStop,
  };

  CartPoleBalanceController(Axis& axis, As5600Sensor& angleSensor);

  void begin();
  bool start();
  void stop(StopReason reason);
  void update();
  void calibrateDownAngle(float angleDeg);
  void printStatus() const;

  bool isActive() const;
  Mode mode() const;
  const char* modeName() const;
  StopReason stopReason() const;
  const char* stopReasonName() const;
  float angleDownDeg() const;

 private:
  struct Observation {
    bool ok = false;
    float thetaRad = 0.0F;
    float phiRad = 0.0F;
    float phidotRadS = 0.0F;
    float xM = 0.0F;
    float xdotMps = 0.0F;
  };

  static float wrapPi(float angleRad);
  static float wrapDeg(float angleDeg);
  static float clip(float value, float low, float high);
  float poleMassKg() const;
  float poleComM() const;
  float poleInertiaKgM2() const;
  float energy(float phiRad, float phidotRadS) const;
  Observation readObservation(float dtS);
  float computeAccel(const Observation& obs);
  void integrateCommand(float accelMps2, float dtS);
  bool emitDueSteps(uint32_t nowUs);
  float commandLagM() const;

  Axis& axis_;
  As5600Sensor& angleSensor_;
  bool active_ = false;
  Mode mode_ = Mode::Swingup;
  StopReason stopReason_ = StopReason::None;
  float angleDownDeg_ = BeltCartpoleConfig::kRealAngleDownDeg;
  float lastThetaRad_ = 0.0F;
  float filteredPhidotRadS_ = 0.0F;
  float lastXM_ = 0.0F;
  float xdotMps_ = 0.0F;
  float cmdPosM_ = 0.0F;
  float cmdVelMps_ = 0.0F;
  float lastAccelMps2_ = 0.0F;
  uint8_t sensorFaultCount_ = 0;
  uint32_t lastControlUs_ = 0;
  uint32_t lastStepUs_ = 0;
  uint32_t lastLogMs_ = 0;
};
