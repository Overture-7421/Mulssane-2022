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
#include <iostream>

void Robot::RobotInit() {

  // m_led.SetLength(kLength);
  // m_led.SetData(m_ledBuffer);
  // m_led.Start();


  // chassis.SetDefaultCommand(drive);
  // chassis.resetOdometry({7.74_m, 2.48_m, {-91.5_deg}});

  storageAndDeliver.SetDefaultCommand(PreloadBall(&storageAndDeliver, &omnisMotor).Perpetually());
  // climber.SetDefaultCommand(SetClimberVoltage(&climber, 0.0).Perpetually());

  intakeButton.WhileHeld(SetIntake(&intake, 12, true))
    .WhenReleased(frc2::SequentialCommandGroup(SetIntake(&intake, 12, false),
      frc2::WaitCommand(0.2_s),
      SetIntake(&intake, 0, false)));

  // feederShootButton.WhileHeld(SetStorageAndDeliver(&storageAndDeliver, 12)).WhenReleased(SetStorageAndDeliver(&storageAndDeliver, 0));
  // spitBallsTrigger.WhileActiveContinous(
  //   SetStorageAndDeliver(&storageAndDeliver, -12)
  // ).WhenInactive(SetStorageAndDeliver(&storageAndDeliver, 0));
  //  shootLongRangeButton.WhileHeld(SetShooterWithVision(&shooter, &visionManager))
  //      .WhenReleased(SetShooter(&shooter, 0.0));

  //  shootShortRangeButton.WhileHeld(SetShooter(&shooter, 240.0))
  //      .WhenReleased(SetShooter(&shooter, 0.0));

  //   shootLowGoalButton.WhileActiveContinous(SetShooter(&shooter, 120.0)).WhenInactive(SetShooter(&shooter, 0.0));  

  // climberButtonUp.WhenPressed(SetClimberPistonsUp(&intake, &climber))
  //     .WhenReleased(SetClimberPistonsDown(&climber, &intake));

  // climberButtonMotorEnable
  //     .WhileHeld(
  //         [climber = &climber, intake = &intake, joy2 = &joy2] {
  //           double voltage =
  //               (joy2->GetRawAxis(2) * 12.0) - (joy2->GetRawAxis(3) * 12.0);
  //           climber->setVoltage(voltage);
  //           intake->setPistons(true);
  //         },
  //         {&climber, &intake})
  //     .WhenReleased(frc2::ParallelCommandGroup(SetClimberVoltage(&climber, 0)));

  //   // shooter.SetDefaultCommand(SetShooterWithVision(&shooter, &visionManager).Perpetually());

  //   autoChooser.AddOption("Left 2 Ball Auto", &left2BallAuto);
  //   autoChooser.AddOption("Single Center Ball", ¢erSingleBallAuto);
  //   autoChooser.AddOption("Left Kidnap", &leftKidnap);
  //   autoChooser.SetDefaultOption("Right 3 Ball auto", &right3BallAuto);
  //   frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  //   frc::SmartDashboard::PutNumber("ShooterVel", 0.0);
  //   frc::SmartDashboard::PutBoolean("HoodState", false);



}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {
  //autocommand = std::make_unique<Right_4BallAuto>(&chassis, &visionManager);
 //  autoChooser.GetSelected()->Schedule();
}

void Robot::AutonomousPeriodic() {
  // hood.MotorMovement(-3_V);
}

void Robot::TeleopInit() {
  // visionManager.setLeds(true);
  // frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TeleopPeriodic() {
}

void Robot::DisabledInit() {
  //   visionManager.setLeds(false);
  // frc2::CommandScheduler::GetInstance().CancelAll();
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






// No fue facil, pero nos llevo al mundial y nos dejo en el ranking 29, se agradece infinitamente, gracias por compliar, gracias por avisarnos de errores, gracias por ser tan C++. 
// Mulssane es un gran robot y su programa lo es tambien, excelente trabajo. 
