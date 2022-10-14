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
#include "Commands/Common/SetShooterWithVision/SetShooterWithVision.h"
#include "Commands/Common/SetStorageAndDeliver/SetStorageAndDeliver.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Right_3BallAuto::Right_3BallAuto(Chassis* chassis, VisionManager* visionManager,
                                 Intake* intake,
                                 StorageAndDeliver* storageAndDeliver,
                                 Shooter* shooter, Hood* hood) {
  // Add your commands here, e.g.
  AddCommands(frc2::ParallelDeadlineGroup(
      frc2::SequentialCommandGroup(
          frc2::InstantCommand(
              [chassis = chassis, visionManager = visionManager] {
                chassis->resetOdometry({7.74_m, 2.48_m, {-91.5_deg}});
                visionManager->setLeds(false);
              },
              {chassis}),
          SetIntake(intake, 12, true),
          frc2::ParallelDeadlineGroup(
              frc2::SequentialCommandGroup(
                  chassis->getRamseteCommand(
                      {{7.74_m, 2.48_m, -91.5_deg}, {7.7_m, 1.6_m, -90_deg}},
                      {2.0_mps, 2.0_mps_sq}),
                  frc2::WaitCommand(0.1_s), SetIntake(intake, 12, false)),

              PreloadBall(storageAndDeliver).Perpetually()),
          frc2::InstantCommand([visionManager = visionManager] {
            visionManager->setLeds(true);
          }),
          frc2::WaitCommand(0.5_s),
          frc2::ParallelCommandGroup(AlignToTower(chassis, visionManager),
                                     SetIntake(intake, 0, false)),
          frc2::WaitCommand(0.5_s),
          AutoShoot(chassis, storageAndDeliver, visionManager, 1)
              .WithTimeout(4_s),
          frc2::WaitCommand(0.5_s),
          AutoShoot(chassis, storageAndDeliver, visionManager, 1)
              .WithTimeout(4_s),
          frc2::InstantCommand([visionManager = visionManager] {
            visionManager->setLeds(false);
          }),
          TurnToAngle(chassis, 160), SetIntake(intake, 12, true),
          frc2::ParallelDeadlineGroup(
              frc2::SequentialCommandGroup(
                  chassis->getRamseteCommand(
                      {{7.6_m, 1.6_m, 160_deg}, {5.0_m, 3.0_m, 90_deg}},
                      {2.0_mps, 2.0_mps_sq}),
                  frc2::WaitCommand(0.1_s), SetIntake(intake, 12, false)),
              PreloadBall(storageAndDeliver).Perpetually()),
          SetStorageAndDeliver(storageAndDeliver, 0.0),
          TurnToAngle(chassis, 180),
          frc2::InstantCommand([visionManager = visionManager, storageAndDeliver = storageAndDeliver] {
            visionManager->setLeds(true);
                      }),
          frc2::WaitCommand(0.5_s),
          frc2::ParallelCommandGroup(AlignToTower(chassis, visionManager),
                                     SetIntake(intake, 0, false)),
          frc2::WaitCommand(1.5_s),
          AutoShoot(chassis, storageAndDeliver, visionManager, 1)
              .WithTimeout(4_s),
          frc2::WaitCommand(0.5_s),
          AutoShoot(chassis, storageAndDeliver, visionManager, 1)
              .WithTimeout(4_s)
              ),
      SetShooterWithVision(shooter, hood, visionManager).Perpetually())

  );
}
