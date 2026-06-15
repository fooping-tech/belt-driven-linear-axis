#include "CartPoleBalanceController.h"

#include <math.h>

namespace {
constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;
constexpr float kDegToRad = kPi / 180.0F;
constexpr float kGravity = 9.81F;
constexpr float kMinStepSpeedMps = 0.0005F;
}

CartPoleBalanceController::CartPoleBalanceController(Axis& axis, As5600Sensor& angleSensor)
    : axis_(axis), angleSensor_(angleSensor) {}

void CartPoleBalanceController::begin() {
  active_ = false;
  stopReason_ = StopReason::None;
}

bool CartPoleBalanceController::start() {
  if (!axis_.isHomed()) {
    stopReason_ = StopReason::NotHomed;
    Serial.println("BALANCE,rejected=1,reason=not_homed");
    return false;
  }

  const As5600Sensor::Reading reading = angleSensor_.read();
  if (!reading.ok || !As5600Sensor::magnetDetected(reading.status)) {
    stopReason_ = StopReason::SensorFault;
    Serial.printf("BALANCE,rejected=1,reason=sensor_fault,as5600_ok=%u,magnet=%u\n",
                  reading.ok ? 1 : 0,
                  reading.statusOk && As5600Sensor::magnetDetected(reading.status) ? 1 : 0);
    return false;
  }

  active_ = true;
  mode_ = Mode::Swingup;
  stopReason_ = StopReason::None;
  filteredPhidotRadS_ = 0.0F;
  cmdVelMps_ = 0.0F;
  lastAccelMps2_ = 0.0F;
  cmdPosM_ = (axis_.currentPositionMm() - BeltCartpoleConfig::kRealXCenterMm) / 1000.0F;
  lastXM_ = cmdPosM_;
  const float thetaRad = wrapPi(BeltCartpoleConfig::kRealAngleDirection
                                * wrapDeg(reading.angleDegrees - angleDownDeg_) * kDegToRad);
  lastThetaRad_ = thetaRad;
  const uint32_t nowUs = micros();
  lastControlUs_ = nowUs;
  lastStepUs_ = nowUs;
  lastLogMs_ = millis();
  Serial.printf("BALANCE,start,x_mm=%.3f,x_center_mm=%.3f,angle_down_deg=%.2f,theta_rad=%.4f\n",
                axis_.currentPositionMm(),
                BeltCartpoleConfig::kRealXCenterMm,
                angleDownDeg_,
                thetaRad);
  return true;
}

void CartPoleBalanceController::stop(StopReason reason) {
  if (!active_ && reason == StopReason::None) {
    return;
  }
  active_ = false;
  stopReason_ = reason;
  cmdVelMps_ = 0.0F;
  lastAccelMps2_ = 0.0F;
  Serial.printf("BALANCE,stop,reason=%s,x_mm=%.3f,cmd_x_m=%.5f,mode=%s\n",
                stopReasonName(),
                axis_.currentPositionMm(),
                cmdPosM_,
                modeName());
}

void CartPoleBalanceController::update() {
  if (!active_) {
    return;
  }

  const uint32_t nowUs = micros();
  const float elapsedS = static_cast<float>(nowUs - lastControlUs_) / 1000000.0F;
  if (elapsedS >= BeltCartpoleConfig::kCtrlDtS) {
    lastControlUs_ = nowUs;
    Observation obs = readObservation(elapsedS);
    if (!obs.ok) {
      stop(StopReason::SensorFault);
      return;
    }

    const float accel = computeAccel(obs);
    integrateCommand(accel, elapsedS);
    lastAccelMps2_ = accel;

    const uint32_t nowMs = millis();
    if (nowMs - lastLogMs_ >= BeltCartpoleConfig::kBalanceLogIntervalMs) {
      lastLogMs_ = nowMs;
      Serial.printf("BALANCE_STATUS,mode=%s,x_m=%.5f,xdot_mps=%.4f,phi_rad=%.4f,phidot_rad_s=%.4f,cmd_x_m=%.5f,cmd_v_mps=%.4f,accel_mps2=%.3f,cmd_lag_m=%.5f\n",
                    modeName(),
                    obs.xM,
                    obs.xdotMps,
                    obs.phiRad,
                    obs.phidotRadS,
                    cmdPosM_,
                    cmdVelMps_,
                    lastAccelMps2_,
                    commandLagM());
    }
  }

  if (!emitDueSteps(nowUs)) {
    return;
  }
}

void CartPoleBalanceController::calibrateDownAngle(float angleDeg) {
  angleDownDeg_ = wrapDeg(angleDeg);
  Serial.printf("BALANCE,angle_down_deg=%.2f\n", angleDownDeg_);
}

void CartPoleBalanceController::printStatus() const {
  Serial.printf("BALANCE_STATUS,active=%u,mode=%s,stop_reason=%s,angle_down_deg=%.2f,x_center_mm=%.3f,x_lim_m=%.3f,v_max_mps=%.3f,a_max_mps2=%.3f,ctrl_hz=%.1f\n",
                active_ ? 1 : 0,
                modeName(),
                stopReasonName(),
                angleDownDeg_,
                BeltCartpoleConfig::kRealXCenterMm,
                BeltCartpoleConfig::kXLimM,
                BeltCartpoleConfig::kVMaxMps,
                BeltCartpoleConfig::kAMaxMps2,
                BeltCartpoleConfig::kCtrlHz);
}

bool CartPoleBalanceController::isActive() const {
  return active_;
}

CartPoleBalanceController::Mode CartPoleBalanceController::mode() const {
  return mode_;
}

const char* CartPoleBalanceController::modeName() const {
  switch (mode_) {
    case Mode::Swingup:
      return "swingup";
    case Mode::Lqr:
      return "lqr";
  }
  return "unknown";
}

CartPoleBalanceController::StopReason CartPoleBalanceController::stopReason() const {
  return stopReason_;
}

const char* CartPoleBalanceController::stopReasonName() const {
  switch (stopReason_) {
    case StopReason::None:
      return "none";
    case StopReason::NotHomed:
      return "not_homed";
    case StopReason::SensorFault:
      return "sensor_fault";
    case StopReason::MotorFault:
      return "motor_fault";
    case StopReason::SoftLimit:
      return "soft_limit";
    case StopReason::Stall:
      return "stall";
    case StopReason::UserStop:
      return "user_stop";
  }
  return "unknown";
}

float CartPoleBalanceController::angleDownDeg() const {
  return angleDownDeg_;
}

float CartPoleBalanceController::wrapPi(float angleRad) {
  while (angleRad > kPi) {
    angleRad -= kTwoPi;
  }
  while (angleRad < -kPi) {
    angleRad += kTwoPi;
  }
  return angleRad;
}

float CartPoleBalanceController::wrapDeg(float angleDeg) {
  while (angleDeg >= 360.0F) {
    angleDeg -= 360.0F;
  }
  while (angleDeg < 0.0F) {
    angleDeg += 360.0F;
  }
  return angleDeg;
}

float CartPoleBalanceController::clip(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

float CartPoleBalanceController::poleMassKg() const {
  return BeltCartpoleConfig::kRodMassKg + BeltCartpoleConfig::kTipMassKg;
}

float CartPoleBalanceController::poleComM() const {
  const float m = poleMassKg();
  return (BeltCartpoleConfig::kRodMassKg * BeltCartpoleConfig::kPoleLenM * 0.5F
          + BeltCartpoleConfig::kTipMassKg * BeltCartpoleConfig::kPoleLenM) / m;
}

float CartPoleBalanceController::poleInertiaKgM2() const {
  return BeltCartpoleConfig::kRodMassKg * BeltCartpoleConfig::kPoleLenM * BeltCartpoleConfig::kPoleLenM / 3.0F
         + BeltCartpoleConfig::kTipMassKg * BeltCartpoleConfig::kPoleLenM * BeltCartpoleConfig::kPoleLenM;
}

float CartPoleBalanceController::energy(float phiRad, float phidotRadS) const {
  return 0.5F * poleInertiaKgM2() * phidotRadS * phidotRadS
         + poleMassKg() * kGravity * poleComM() * (cosf(phiRad) - 1.0F);
}

CartPoleBalanceController::Observation CartPoleBalanceController::readObservation(float dtS) {
  Observation obs;
  const As5600Sensor::Reading reading = angleSensor_.read();
  if (!reading.ok || !As5600Sensor::magnetDetected(reading.status)) {
    return obs;
  }

  obs.thetaRad = wrapPi(BeltCartpoleConfig::kRealAngleDirection
                        * wrapDeg(reading.angleDegrees - angleDownDeg_) * kDegToRad);
  obs.phiRad = wrapPi(obs.thetaRad - kPi);
  const float rawPhidot = wrapPi(obs.thetaRad - lastThetaRad_) / dtS;
  filteredPhidotRadS_ += BeltCartpoleConfig::kAngleFilterAlpha * (rawPhidot - filteredPhidotRadS_);
  obs.phidotRadS = filteredPhidotRadS_;
  lastThetaRad_ = obs.thetaRad;

  obs.xM = (axis_.currentPositionMm() - BeltCartpoleConfig::kRealXCenterMm) / 1000.0F;
  xdotMps_ = (obs.xM - lastXM_) / dtS;
  lastXM_ = obs.xM;
  obs.xdotMps = xdotMps_;
  obs.ok = true;
  return obs;
}

float CartPoleBalanceController::computeAccel(const Observation& obs) {
  if (mode_ == Mode::Swingup) {
    if (fabsf(obs.phiRad) < BeltCartpoleConfig::kCatchPhiRad
        && fabsf(obs.phidotRadS) < BeltCartpoleConfig::kCatchPhidotRadS) {
      mode_ = Mode::Lqr;
    }
  } else if (fabsf(obs.phiRad) > BeltCartpoleConfig::kReleasePhiRad) {
    mode_ = Mode::Swingup;
  }

  if (mode_ == Mode::Lqr) {
    const float accel = -(BeltCartpoleConfig::kLqrKx * obs.xM
                          + BeltCartpoleConfig::kLqrKxd * obs.xdotMps
                          + BeltCartpoleConfig::kLqrKphi * obs.phiRad
                          + BeltCartpoleConfig::kLqrKphid * obs.phidotRadS);
    return clip(accel, -BeltCartpoleConfig::kAMaxMps2, BeltCartpoleConfig::kAMaxMps2);
  }

  const float e = energy(obs.phiRad, obs.phidotRadS);
  const float eBottom = -2.0F * poleMassKg() * kGravity * poleComM();
  const float swingSat = BeltCartpoleConfig::kSwingSatRatio * BeltCartpoleConfig::kAMaxMps2;
  float accel = 0.0F;
  if (fabsf(e - eBottom) < 0.02F * fabsf(eBottom) && fabsf(obs.phidotRadS) < 0.2F) {
    accel = swingSat;
  } else {
    accel = BeltCartpoleConfig::kKEnergy * e * obs.phidotRadS * cosf(obs.phiRad);
    if (fabsf(obs.phidotRadS) < BeltCartpoleConfig::kDeadlockPhidotRadS
        && fabsf(accel) < BeltCartpoleConfig::kDeadlockAccelMps2) {
      accel = obs.phiRad >= 0.0F
                  ? -BeltCartpoleConfig::kDeadlockAccelMps2
                  : BeltCartpoleConfig::kDeadlockAccelMps2;
    }
  }
  accel += -BeltCartpoleConfig::kCenterHoldKx * obs.xM
           - BeltCartpoleConfig::kCenterHoldKd * obs.xdotMps;
  return clip(accel, -swingSat, swingSat);
}

void CartPoleBalanceController::integrateCommand(float accelMps2, float dtS) {
  float accel = clip(accelMps2, -BeltCartpoleConfig::kAMaxMps2, BeltCartpoleConfig::kAMaxMps2);
  const float limit = BeltCartpoleConfig::kXLimM - BeltCartpoleConfig::kSoftMarginM;
  if (fabsf(cmdVelMps_) > 1.0e-6F) {
    const float stopDist = cmdVelMps_ * cmdVelMps_ / (2.0F * BeltCartpoleConfig::kAMaxMps2);
    if (cmdVelMps_ > 0.0F && cmdPosM_ + stopDist >= limit) {
      accel = -BeltCartpoleConfig::kAMaxMps2;
    } else if (cmdVelMps_ < 0.0F && cmdPosM_ - stopDist <= -limit) {
      accel = BeltCartpoleConfig::kAMaxMps2;
    }
  }

  cmdVelMps_ = clip(cmdVelMps_ + accel * dtS, -BeltCartpoleConfig::kVMaxMps, BeltCartpoleConfig::kVMaxMps);
  cmdPosM_ += cmdVelMps_ * dtS;
  if (cmdPosM_ > BeltCartpoleConfig::kXLimM) {
    cmdPosM_ = BeltCartpoleConfig::kXLimM;
    cmdVelMps_ = 0.0F;
  } else if (cmdPosM_ < -BeltCartpoleConfig::kXLimM) {
    cmdPosM_ = -BeltCartpoleConfig::kXLimM;
    cmdVelMps_ = 0.0F;
  }
}

bool CartPoleBalanceController::emitDueSteps(uint32_t nowUs) {
  if (fabsf(cmdVelMps_) < kMinStepSpeedMps) {
    lastStepUs_ = nowUs;
    return true;
  }

  const long targetSteps = lroundf((BeltCartpoleConfig::kRealXCenterMm + cmdPosM_ * 1000.0F) * axis_.stepsPerMm());
  const float stepRate = fabsf(cmdVelMps_) * 1000.0F * axis_.stepsPerMm();
  if (stepRate <= 0.0F) {
    return true;
  }

  const uint32_t intervalUs = static_cast<uint32_t>(1000000.0F / stepRate);
  const uint32_t safeIntervalUs = intervalUs > 0 ? intervalUs : 1;
  if (nowUs - lastStepUs_ < safeIntervalUs) {
    return true;
  }

  const long delta = targetSteps - axis_.currentPositionSteps();
  if (delta == 0) {
    lastStepUs_ = nowUs;
    return true;
  }

  const int direction = delta > 0 ? 1 : -1;
  if (!axis_.moveOneStep(direction)) {
    stop(axis_.isWithinSoftLimit(axis_.currentPositionMm()) ? StopReason::MotorFault : StopReason::SoftLimit);
    return false;
  }
  lastStepUs_ = nowUs;
  return true;
}

float CartPoleBalanceController::commandLagM() const {
  const float issuedM = (axis_.currentPositionMm() - BeltCartpoleConfig::kRealXCenterMm) / 1000.0F;
  return cmdPosM_ - issuedM;
}
