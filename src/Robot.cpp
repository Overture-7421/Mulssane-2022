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

#include <wpi/PortForwarder.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PerpetualCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include "Commands/Common/PreloadBall/PreloadBall.h"
#include "Commands/Common/SetClimberPistons/SetClimberPistonsDown.h"
#include "Commands/Common/SetClimberPistons/SetClimberPistonsUp.h"
#include "Commands/Common/SetClimberVoltage/SetClimberVoltage.h"
#include "Commands/Common/SetIntake/SetIntake.h"
#include "Commands/Common/SetShooter/SetShooter.h"
#include "Commands/Common/SetStorageAndDeliver/SetStorageAndDeliver.h"
#include "Commands/Common/SetShooterWithVision/SetShooterWithVision.h"
#include "Commands/Common/WaitBeReadyToShoot/WaitBeReadyToShoot.h"

void Robot::RobotInit() {
  chassis.SetDefaultCommand(drive);
  storageAndDeliver.SetDefaultCommand(PreloadBall(&storageAndDeliver).Perpetually());
  climber.SetDefaultCommand(SetClimberVoltage(&climber, 0.0).Perpetually());

  intakeButton.WhileHeld(SetIntake(&intake, 12, true))
      .WhenReleased(frc2::SequentialCommandGroup(SetIntake(&intake, 12, false),
                                                 frc2::WaitCommand(0.2_s),
                                                 SetIntake(&intake, 0, false)));

  feederShootButton.WhileHeld(SetStorageAndDeliver(&storageAndDeliver, 8)).WhenReleased(SetStorageAndDeliver(&storageAndDeliver, 0));

   shootLongRangeButton.WhileHeld(SetShooterWithVision(&shooter, &visionManager))
       .WhenReleased(SetShooter(&shooter, 0.0, true));

   shootShortRangeButton.WhileHeld(SetShooter(&shooter, 360.0, false))
       .WhenReleased(SetShooter(&shooter, 0.0, true));

    shootLowGoalButton.WhileActiveContinous(SetShooter(&shooter, 180.0, true)).WhenInactive(SetShooter(&shooter, 0.0, true));  

  climberButtonUp.WhenPressed(SetClimberPistonsUp(&intake, &climber))
      .WhenReleased(SetClimberPistonsDown(&climber, &intake));

  climberButtonMotorEnable
      .WhileHeld(
          [climber = &climber, intake = &intake, joy2 = &joy2] {
            double voltage =
                (joy2->GetRawAxis(2) * 12.0) - (joy2->GetRawAxis(3) * 12.0);
            climber->setVoltage(voltage);
            intake->setPistons(true);
          },
          {&climber, &intake})
      .WhenReleased(frc2::ParallelCommandGroup(SetClimberVoltage(&climber, 0)));


    autoChooser.AddOption("Left 2 Ball Auto", &left2BallAuto);
    autoChooser.SetDefaultOption("Right 3 Ball auto", &right3BallAuto);
    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);


}

void Robot::RobotPeriodic() {
  rangeDecider.updateRangeDecision(chassis.getPose(),
                                   visionManager.getTargetPose());
  frc::SmartDashboard::PutNumber("Abs Position", absEnc.GetAbsolutePosition());

  //shooter.setVelocity(frc::SmartDashboard::GetNumber("ShooterVel", 0.0));
  //shooter.setHoodState(frc::SmartDashboard::GetBoolean("HoodState", false));
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {
   //autocommand = std::make_unique<Right_4BallAuto>(&chassis, &visionManager);
   autoChooser.GetSelected()->Schedule();
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  visionManager.setLeds(true);
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
    visionManager.setLeds(false);
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
int main() {
  photonlib::PhotonCamera::SetVersionCheckEnabled(false);
  wpi::PortForwarder::GetInstance().Add(5800, "10.74.21.6", 5800);

  // These warnings generate console prints that cause scheduling jitter
  frc::DriverStation::SilenceJoystickConnectionWarning(true);
  // This telemetry regularly causes loop overruns
  frc::LiveWindow::DisableAllTelemetry();

  return frc::StartRobot<Robot>();
}
#endif
