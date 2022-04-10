// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/VisionManager/VisionManager.h"
#include "Subsystems/RangeDecider/RangeDecider.h"
#include <cmath>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DefaultDrive
    : public frc2::CommandHelper<frc2::CommandBase, DefaultDrive> {
 public:
  DefaultDrive(Chassis* chassis, VisionManager* visionManager, RangeDecider* rangeDecider, frc::Joystick* joy);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
    Chassis* chassis;
    VisionManager* visionManager;

    RangeDecider* rangeDecider;
    frc::Joystick* joy;

    const int aimButton = 6;
    frc::SlewRateLimiter<units::meters_per_second> linearLimiter{6_mps_sq};
    frc::ProfiledPIDController<units::degrees> headingController {0.065, 0, 0.0, {units::degrees_per_second_t(360 * 1.5), units::degrees_per_second_squared_t(360 * 0.75)}};
};
