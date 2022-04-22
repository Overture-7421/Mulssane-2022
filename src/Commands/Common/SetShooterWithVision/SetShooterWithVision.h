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
    {1.5, 345},
    {2.0, 324},
    {2.3, 340},
    {2.6, 343},
    {2.9, 355},
    {3.2, 390}
    //{3.8, 395},
    //{4.1, 427},
    //{4.5, 405} 
    //{4.6, 460},
    //{4.7, 412},
    //{4.8, 420},
    //{5.0, 430}, //HOOD
    //{5.3, 438},
    //{5.5, 440},
    //{5.7, 446},
    //{6.0, 447},
    //{6.3, 450},
    //{6.6, 465}
  }};
};
