// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Subsystems/Chassis/Chassis.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TurnToAngle
    : public frc2::CommandHelper<frc2::CommandBase, TurnToAngle> {
 public:
  TurnToAngle(Chassis* chassis, double angleObjective);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  double angleObjective;
  Chassis* chassis;
  // frc2::PIDController turnToAnglePID {0.1, 0.5, 0.0002};

  frc::ProfiledPIDController<units::degrees> turnToAnglePID {0.07, 0, 0.0003, {units::degrees_per_second_t(360 * 1.5), units::degrees_per_second_squared_t(360 * 6)}};
};
