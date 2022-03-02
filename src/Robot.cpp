/*
    ____                     __     ____        __          __     _   __
   /  _/___  ________  _____/ /_   / __ \____  / /_  ____  / /_   / | / /___
_____ ___  ___
   / // __ \/ ___/ _ \/ ___/ __/  / /_/ / __ \/ __ \/ __ \/ __/  /  |/ / __ `/
__ `__ \/ _ \
 _/ // / / (__  )  __/ /  / /_   / _, _/ /_/ / /_/ / /_/ / /_   / /|  / /_/ / /
/ / / /  __/
/___/_/ /_/____/\___/_/   \__/  /_/ |_|\____/_.___/\____/\__/  /_/ |_/\__,_/_/
/_/ /_/\___/
*/

#include "Robot.h"

#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include <iostream>

#include "Commands/Common/PreloadBall/PreloadBall.h"
#include "Commands/Common/SetClimberPistons/SetClimberPistonsDown.h"
#include "Commands/Common/SetClimberPistons/SetClimberPistonsUp.h"
#include "Commands/Common/SetClimberVoltage/SetClimberVoltage.h"
#include "Commands/Common/SetIntake/SetIntake.h"
#include "Commands/Common/SetShooter/SetShooter.h"
#include "Commands/Common/SetStorageAndDeliver/SetStorageAndDeliver.h"

void Robot::RobotInit() {
  chassis.SetDefaultCommand(drive);
  storageAndDeliver.SetDefaultCommand(PreloadBall(&storageAndDeliver, true));

  intakeButton.WhileHeld(SetIntake(&intake, 12, true))
      .WhenReleased(frc2::SequentialCommandGroup(SetIntake(&intake, 12, false),
                                                 frc2::WaitCommand(0.2_s),
                                                 SetIntake(&intake, 0, false)));

  shootButton.WhileHeld(SetStorageAndDeliver(&storageAndDeliver, 12))
      .WhenReleased(SetStorageAndDeliver(&storageAndDeliver, 0));

  driverShootButton.WhenPressed(SetShooter(&shooter, 370.0, true))
      .WhenReleased(SetShooter(&shooter, 0.0, true));

  driverShootNoVisionButton.WhenPressed(SetShooter(&shooter, 360.0, false))
      .WhenReleased(SetShooter(&shooter, 0.0, true));

  // climberButtonUp.WhenPressed(SetClimberPistonsUp(&intake, &climber))
  //     .WhenReleased(SetClimberPistonsDown(&intake, &climber));

  // climberButtonMotorEnable
  //     .WhileHeld(
  //         [climber = &climber, intake = &intake, joy2 = &joy2] {
  //           double voltage =
  //               (joy2->GetRawAxis(2) * 12.0) - (joy2->GetRawAxis(3) * 12.0);
  //           climber->setVoltage(voltage);
  //           intake->setPistons(true);
  //         },
  //         {&climber, &intake})
  //     .WhenReleased(frc2::ParallelCommandGroup(SetClimberVoltage(&climber, 0),
  //                                              SetIntake(&intake, 0, false)));
}

void Robot::RobotPeriodic() {
  rangeDecider.updateRangeDecision(chassis.getPose(),
                                   visionManager.getTargetPose());

  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {
  // autocommand = std::make_unique<RamseteTests>(&chassis);
  // autocommand->Schedule();
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  visionManager.setLeds(true);
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
  visionManager.setLeds(true);
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
int main() {
  // These warnings generate console prints that cause scheduling jitter
  frc::DriverStation::SilenceJoystickConnectionWarning(true);
  // This telemetry regularly causes loop overruns
  frc::LiveWindow::DisableAllTelemetry();

  return frc::StartRobot<Robot>();
}
#endif
