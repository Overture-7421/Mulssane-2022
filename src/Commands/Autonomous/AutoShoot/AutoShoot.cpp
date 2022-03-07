// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AutoShoot.h"

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/PerpetualCommand.h>
#include <frc2/command/WaitCommand.h>

#include "Commands/Autonomous/AlignToTower/AlignToTower.h"
#include "Commands/Autonomous/FeedNBalls/FeedNBalls.h"
#include "Commands/Common/SetStorageAndDeliver/SetStorageAndDeliver.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
AutoShoot::AutoShoot(Chassis* chassis, StorageAndDeliver* storageAndDeliver,
                     VisionManager* visionManager, int ballsToShoot)
    : CommandHelper(frc2::SequentialCommandGroup(
          FeedNBalls(storageAndDeliver, ballsToShoot),
          frc2::WaitCommand(0.5_s),
          SetStorageAndDeliver(storageAndDeliver, 0.0))) {

  AddCommands(AlignToTower(chassis, visionManager).Perpetually());
}
