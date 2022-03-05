// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include "Subsystems/StorageAndDeliver/StorageAndDeliver.h"
#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/VisionManager/VisionManager.h"

class AutoShoot
    : public frc2::CommandHelper<frc2::ParallelDeadlineGroup,
                                 AutoShoot> {
 public:
  AutoShoot(Chassis* chassis, StorageAndDeliver* storageAndDeliver, VisionManager* visionManager, int ballsToShoot);

};
