#pragma once

// Generated from sim_config/belt_cartpole.json. Do not edit by hand.
namespace BeltCartpoleConfig {
constexpr float kBeltPitchM = 0.002F;
constexpr int kPulleyTeeth = 20;
constexpr float kXLimM = 0.16F;
constexpr float kSoftMarginM = 0.005F;
constexpr float kVMaxMps = 0.3F;
constexpr float kAMaxMps2 = 4.0F;
constexpr float kStallThresholdM = 0.004F;
constexpr float kPoleLenM = 0.32F;
constexpr float kRodMassKg = 0.045F;
constexpr float kTipMassKg = 0.0F;
constexpr float kHingeDampingNms = 0.00015F;
constexpr float kCtrlHz = 50.0F;
constexpr float kCtrlDtS = 1.0F / kCtrlHz;
constexpr float kKEnergy = 420.0F;
constexpr float kSwingSatRatio = 1.0F;
constexpr float kDeadlockPhidotRadS = 0.08F;
constexpr float kDeadlockAccelMps2 = 0.4F;
constexpr float kCenterHoldKx = 4.0F;
constexpr float kCenterHoldKd = 2.0F;
constexpr float kSwingRailGuardXM = 0.1F;
constexpr float kSwingTopBrakePhiRad = 1.3F;
constexpr float kSwingTopBrakePhidotRadS = 1.5F;
constexpr float kSwingTopBrakeRatio = 0.6F;
constexpr float kCatchPhiRad = 0.5F;
constexpr float kCatchPhidotRadS = 6.0F;
constexpr float kReleasePhiRad = 0.8F;
constexpr float kLqrKx = -22.3606798F;
constexpr float kLqrKxd = -26.0060218F;
constexpr float kLqrKphi = -147.196973F;
constexpr float kLqrKphid = -18.237235F;
constexpr float kLqrCenterKx = 20.0F;
constexpr float kLqrCenterKd = 5.0F;
constexpr float kRealXMinMm = 0.0F;
constexpr float kRealXCenterMm = 177.5F;
constexpr float kRealAngleDownDeg = 0.0F;
constexpr int kRealAngleDirection = 1;
constexpr float kAngleFilterAlpha = 0.35F;
constexpr unsigned int kBalanceCurrentMa = 1400;
constexpr bool kBalanceSpreadCycle = true;
constexpr unsigned long kBalanceLogIntervalMs = 100UL;
constexpr unsigned int kMaxStepsPerUpdate = 64;
}  // namespace BeltCartpoleConfig
