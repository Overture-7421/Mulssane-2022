// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Right_3BallAuto.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/PerpetualCommand.h>
#include <frc2/command/WaitCommand.h>

#include "Commands/Autonomous/AlignToTower/AlignToTower.h"
#include "Commands/Autonomous/TurnToAngle/TurnToAngle.h"
#include "Commands/Common/PreloadBall/PreloadBall.h"
#include "Commands/Common/SetIntake/SetIntake.h"
#include "Commands/Common/SetShooter/SetShooter.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Right_3BallAuto::Right_3BallAuto(Chassis* chassis, VisionManager* visionManager,
                                 Intake* intake,
                                 StorageAndDeliver* storageAndDeliver,
                                 Shooter* shooter) {
  // Add your commands here, e.g.
  AddCommands(
      frc2::InstantCommand(
          [chassis = chassis] {
            chassis->resetOdometry({7.74_m, 2.48_m, {-91.5_deg}});
          },
          {chassis}),
      SetIntake(intake, 12, true),
      frc2::ParallelDeadlineGroup(
          frc2::SequentialCommandGroup(
              chassis->getRamseteCommand(
                  {{7.74_m, 2.48_m, -91.5_deg}, {7.6_m, 1.6_m, -90_deg}},
                  {2.5_mps, 2.5_mps_sq}),
              SetShooter(shooter, 380, true),
              frc2::ParallelCommandGroup(
                  frc2::SequentialCommandGroup(
                      frc2::WaitCommand(0.1_s), SetIntake(intake, 12, false),
                      frc2::WaitCommand(0.5_s), SetIntake(intake, 0, false)),
                  TurnToAngle(chassis, 90))),

          PreloadBall(storageAndDeliver).Perpetually()),

      frc2::InstantCommand(
          [visionManager = visionManager] { visionManager->setLeds(true); }),
      AlignToTower(chassis, visionManager),
      AutoShoot(chassis, storageAndDeliver, visionManager, 2).WithTimeout(4_s),
      frc2::InstantCommand(
          [visionManager = visionManager] { visionManager->setLeds(false); }),
      SetIntake(intake, 12, true),
      chassis->getRamseteCommand(
          {{7.6_m, 1.75_m, 90_deg}, {5.15_m, 3_m, 180_deg}},
          {2.5_mps, 2.5_mps_sq}),
      SetShooter(shooter, 350, true),

      frc2::ParallelCommandGroup(
          frc2::SequentialCommandGroup(
              frc2::WaitCommand(0.1_s), SetIntake(intake, 12, false),
              frc2::WaitCommand(0.5_s), SetIntake(intake, 0, false)),
          TurnToAngle(chassis, 0)),

      frc2::ParallelDeadlineGroup(
          frc2::SequentialCommandGroup(chassis->getRamseteCommand(
              {{5.15_m, 3_m, 0_deg}, {6.5_m, 2.75_m, 0_deg}},
              {2.5_mps, 2.5_mps_sq})),
          PreloadBall(storageAndDeliver).Perpetually()),
        frc2::InstantCommand(
        [visionManager = visionManager] { visionManager->setLeds(true); }),
      AlignToTower(chassis, visionManager),
      AutoShoot(chassis, storageAndDeliver, visionManager, 1).WithTimeout(4_s)

  );
}
