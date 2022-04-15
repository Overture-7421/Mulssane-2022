// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/VisionManager/VisionManager.h"
#include "Utils/Interpolation/LinearInterpolator/LinearInterpolator.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SetShooterWithVision
    : public frc2::CommandHelper<frc2::CommandBase, SetShooterWithVision> {
 public:
  SetShooterWithVision(Shooter* shooter, VisionManager* visionManager);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Shooter* shooter;
  VisionManager* visionManager;
  LinearInterpolator distanceVsVelocityInterpolator {{
    {2.0, 324},
    {2.3, 340},
    {2.6, 343},
    {2.9, 355},
    {3.2, 367},
    {3.5, 372},
    {3.7, 380},
    {3.8, 395},
    {4.1, 427},
    {4.4, 450},
    {4.5, 396}, //HOOD ARRIBA
    {4.55, 400},
    {4.70, 412},
    {4.85, 420},
    {5.00, 430},
    {5.15, 432},
    {5.30, 438},
    {5.45, 440},
    {5.75, 446},
    {6.05, 455},
    {6.35, 457}
  }};
};
