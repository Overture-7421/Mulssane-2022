// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <photonlib/PhotonCamera.h>

#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/VisionManager/VisionManager.h"

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
  AlignToTower(Chassis* chassis, VisionManager* visionManager);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Chassis* chassis;
  VisionManager* visionManager;

  frc::ProfiledPIDController<units::degrees> turnToAnglePID {0.125, 0, 0.0, {units::degrees_per_second_t(360 * 2.5), units::degrees_per_second_squared_t(360 * 1.0)}};
};
