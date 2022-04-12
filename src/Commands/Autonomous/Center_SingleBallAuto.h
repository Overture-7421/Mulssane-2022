// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/VisionManager/VisionManager.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Commands/Autonomous/AlignToTower/AlignToTower.h"
#include "Commands/Autonomous/FeedNBalls/FeedNBalls.h"
#include "Commands/Autonomous/AutoShoot/AutoShoot.h"


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Center_SingleBallAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, 
                                Center_SingleBallAuto> {
 public:
  Center_SingleBallAuto(Chassis* chassis, VisionManager* visionManager, Intake* intake, StorageAndDeliver* storageAndDeliver, Shooter* shooter);
};
