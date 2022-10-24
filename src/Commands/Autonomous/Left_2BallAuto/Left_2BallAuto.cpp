// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Left_2BallAuto.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PerpetualCommand.h>

#include "Commands/Autonomous/TurnToAngle/TurnToAngle.h"
#include "Commands/Common/SetIntake/SetIntake.h"
#include "Commands/Common/SetShooter/SetShooter.h"
#include "Commands/Common/SetHood/SetHood.h"
#include "Commands/Common/SetStorageAndDeliver/SetStorageAndDeliver.h"
#include "Commands/Common/AutoPreloadBall/AutoPreloadBall.h"


// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Left_2BallAuto::Left_2BallAuto(Chassis* chassis, VisionManager* visionManager,
                               Intake* intake,
                               StorageAndDeliver* storageAndDeliver,
                               Shooter* shooter, Hood* hood) {
  // Add your commands here, e.g.
  AddCommands(frc2::InstantCommand(
                  [chassis = chassis, visionManager = visionManager] {
                    chassis->resetOdometry({5.96_m, 5.34_m, 135_deg});
                    visionManager->setLeds(false);
                  },
                  {chassis}),
              frc2::ParallelDeadlineGroup(
                  frc2::SequentialCommandGroup(
                      SetIntake(intake, 12, true),
                      chassis->getRamseteCommand(
                          {{5.96_m, 5.34_m, 135_deg}, {5.2_m, 6.0_m, 135_deg}},
                          {2.5_mps, 2.5_mps_sq}),
                      SetShooter(shooter, 265), SetHood(hood, 0.25), frc2::WaitCommand(0.6_s),
                      SetIntake(intake, 12, false), frc2::WaitCommand(0.6_s),
                      SetIntake(intake, 0, false)),
                  AutoPreloadBall(storageAndDeliver).Perpetually()),

              frc2::InstantCommand([visionManager = visionManager] {
                visionManager->setLeds(true);
              }), AlignToTower(chassis, visionManager),
                  frc2::WaitCommand(1.5_s),
                  AutoShoot(chassis, storageAndDeliver, visionManager, 1),
                  frc2::WaitCommand(1.5_s),
                  AutoShoot(chassis, storageAndDeliver, visionManager, 1),
                  frc2::WaitCommand(1.5_s),
                  AutoShoot(chassis, storageAndDeliver, visionManager, 1),
                      chassis->getRamseteCommand(
                          {{5.96_m, 5.34_m, 135_deg}, {5.8_m, 6.5_m, 135_deg}},
                          {2.5_mps, 2.5_mps_sq})
  );
}
