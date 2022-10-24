// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/VisionManager/VisionManager.h"
#include "Subsystems/Hood/Hood.h"

class Right_4BallAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 Right_4BallAuto> {
 public:
  Right_4BallAuto(Chassis* chassis, VisionManager* visionManager, Hood* hood);
};
