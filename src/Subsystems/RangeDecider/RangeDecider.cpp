// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RangeDecider.h"

RangeDecider::RangeDecider() {
  shortRange.minDistance = 0_m;
  shortRange.maxDistance = 1.5_m;

  longRange.minDistance = 1.0_m;
  longRange.maxDistance = 3.5_m;

  currentRange = shortRange;
}

void RangeDecider::updateRangeDecision(const frc::Pose2d& currentPose,
                                       const frc::Pose2d& targetPose) {

  units::meter_t distance = currentPose.Translation().Distance(targetPose.Translation());

  switch (currentRangeResult) {
    case RangeResult::Long:
      if (distance < currentRange.minDistance) {
        currentRange = shortRange;
        currentRangeResult = RangeResult::Short;
      }
      break;
    case RangeResult::Short:
      if (distance > currentRange.maxDistance) {
        currentRange = longRange;
        currentRangeResult = RangeResult::Long;
      }
    default:
      break;
  }
}

RangeDecider::RangeResult RangeDecider::getCurrentRange() {
  return currentRangeResult;
}

// This method will be called once per scheduler run
void RangeDecider::Periodic() {

}
