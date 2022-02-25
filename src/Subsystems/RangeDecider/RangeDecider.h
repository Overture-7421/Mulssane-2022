// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/SubsystemBase.h>

class RangeDecider : public frc2::SubsystemBase {
 public:
  enum class RangeResult { Short, Long };

  struct Range {
    units::meter_t minDistance;
    units::meter_t maxDistance;
  };

  RangeDecider();

  void updateRangeDecision(const frc::Pose2d& currentPose,
                           const frc::Pose2d& targetPose);

  RangeResult getCurrentRange();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  Range shortRange, longRange, currentRange;
  RangeResult decidedRange = RangeResult::Short;
  RangeResult currentRangeResult = RangeResult::Short;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
