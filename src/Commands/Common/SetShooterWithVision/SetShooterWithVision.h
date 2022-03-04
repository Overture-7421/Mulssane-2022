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
    {1.5, 300},
    {2.0, 360},
    {2.5, 380},
    {2.7, 367},
    {2.9, 385},
    {3.1, 390},
    {3.3, 395},
    {3.5, 410},
    {3.7, 420},
    {3.9, 433},
    {4.1, 440},
    {4.3, 450},
    {4.5, 460},
    {4.7, 479},
    {4.9, 483}
  }};
};
