// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RamseteTests.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
RamseteTests::RamseteTests(Chassis* chassis) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  
  AddCommands(
    chassis ->getRamseteCommand({

      {0_m, 0_m, 0_deg},
      {6_m, -1_m, 0_deg},
      {7_m, 0_m, 90_deg},
      {6_m, 1_m, 180_deg},
      {3_m, 1_m, 270_deg},
      {3_m, -1_m, 0_deg}
    },{3.5_mps, 2_mps_sq}),

    chassis ->getRamseteCommand({

      {3_m, -0_m, 0_deg},
      {1_m, -1_m, 180_deg},
      {5_m, -1_m, 180_deg}
    },{3.5_mps, 2_mps_sq}, true),

    TurnToAngle(chassis, 90),
    chassis ->getRamseteCommand({

      {5_m, -1_m, 180_deg},
      {1_m, 0_m, 0_deg}
    },{3.5_mps, 2_mps_sq})
  );
}


//David Soni