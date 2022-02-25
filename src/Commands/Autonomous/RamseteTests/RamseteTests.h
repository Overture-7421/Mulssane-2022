// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "Commands/Autonomous/TurnToAngle/TurnToAngle.h"
#include "Subsystems/Chassis/Chassis.h"

class RamseteTests
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 RamseteTests> {
 public:
  RamseteTests(Chassis* chassis);
};
