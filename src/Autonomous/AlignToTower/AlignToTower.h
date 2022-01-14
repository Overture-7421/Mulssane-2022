// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/Chassis.h"
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <photonlib/PhotonCamera.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AlignToTower
    : public frc2::CommandHelper<frc2::CommandBase, AlignToTower> {
 public:
  AlignToTower(Chassis* chassis, double targetObjective);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  double targetObjective;
  Chassis* chassis;
  frc2::PIDController alignToTowerPID {0.1, 0.5, 0.0002};
  double tolerance = 2;
  double currentAlignment;

  photonlib::PhotonCamera camera{"photonvision"};
};
