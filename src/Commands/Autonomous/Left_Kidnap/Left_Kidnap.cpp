// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Left_Kidnap.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PerpetualCommand.h>

#include "Commands/Autonomous/TurnToAngle/TurnToAngle.h"
#include "Commands/Common/PreloadBall/PreloadBall.h"
#include "Commands/Common/SetIntake/SetIntake.h"
#include "Commands/Common/SetShooter/SetShooter.h"
#include "Commands/Common/SetStorageAndDeliver/SetStorageAndDeliver.h"
#include "Commands/Common/SetShooterWithVision/SetShooterWithVision.h"
// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Left_Kidnap::Left_Kidnap(Chassis* chassis, VisionManager* visionManager,
                               Intake* intake,
                               StorageAndDeliver* storageAndDeliver,
                               Shooter* shooter){
// Add your commands here, e.g.
  AddCommands(frc2::ParallelDeadlineGroup(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand(
              [chassis = chassis, visionManager = visionManager] {
                chassis->resetOdometry({5.96_m, 5.34_m, 135_deg});
                visionManager->setLeds(false);
              },
              {chassis}),
          SetIntake(intake, 12, true),
          frc2::ParallelDeadlineGroup(
              frc2::SequentialCommandGroup(
                  chassis->getRamseteCommand(
                      {{5.96_m, 5.34_m, 135_deg}, {5.2_m, 6.0_m, 135_deg}},
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
          AutoShoot(chassis, storageAndDeliver, visionManager, 2)
              .WithTimeout(4_s),
          frc2::InstantCommand([visionManager = visionManager] {
            visionManager->setLeds(false);
          }),
          TurnToAngle(chassis, 160), SetIntake(intake, 12, true),
          frc2::ParallelDeadlineGroup(
              frc2::SequentialCommandGroup(
                  chassis->getRamseteCommand(
                      {{5.2_m, 5.34_m, 135_deg}, {4.7_m, 7.5_m, 90_deg}},
                      {2.0_mps, 2.0_mps_sq}),
                  frc2::WaitCommand(0.1_s), SetIntake(intake, 12, false)),
              SetStorageAndDeliver(storageAndDeliver, 12)),
          TurnToAngle(chassis, 180),

          frc2::InstantCommand([visionManager = visionManager] {
            visionManager->setLeds(true);
          }),
          frc2::WaitCommand(0.5_s),
          AutoShoot(chassis, storageAndDeliver, visionManager, 1)
              .WithTimeout(4_s)

              ),
      SetShooterWithVision(shooter, visionManager).Perpetually())

  );
}
