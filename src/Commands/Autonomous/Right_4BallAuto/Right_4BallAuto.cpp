// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Right_4BallAuto.h"

#include <Commands/Autonomous/TurnToAngle/TurnToAngle.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
//[7.736501,2.485290,89.400005]
// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Right_4BallAuto::Right_4BallAuto(Chassis* chassis,
                                 VisionManager* visionManager) {
  // Add your commands here, e.g.
  AddCommands(frc2::InstantCommand(
                  [chassis = chassis] {
                    chassis->resetOdometry({7.74_m, 2.48_m, {-91.5_deg}});
                  },
                  {chassis}),
              chassis->getRamseteCommand(
                  {{7.74_m, 2.48_m, -91.5_deg}, {7.5_m, 1.75_m, -90_deg}},
                  {2.5_mps, 1.5_mps_sq}),
              TurnToAngle(chassis, 90),
              frc2::InstantCommand([visionManager = visionManager] {
                visionManager->setLeds(true);
              }),
              frc2::WaitCommand(1_s),
              frc2::InstantCommand([visionManager = visionManager] {
                visionManager->setLeds(false);
              }),
              chassis->getRamseteCommand(
                  {{7.5_m, 1.75_m, 90_deg}, {1.35_m, 2.6_m, -135_deg}},
                  {2.5_mps, 1.5_mps_sq}),
              TurnToAngle(chassis, 0),
              frc2::InstantCommand([visionManager = visionManager] {
                visionManager->setLeds(true);
              }),
              chassis->getRamseteCommand(
                  {{1.35_m, 2.6_m, 0_deg}, {6.5_m, 2.0_m, 0_deg}},
                 {2.5_mps, 1.5_mps_sq}));
}
