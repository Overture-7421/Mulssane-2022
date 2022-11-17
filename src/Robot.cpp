#include "Robot.h"

#include <Commands/Common/SetClimber/SetClimber.h>
#include <Commands/Common/SetIntake/SetIntake.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include <iostream>

void Robot::RobotInit() {
  // open intake
  intakeButton.WhenActive(SetIntake(&intake, true, 12))
      .WhenInactive(SetIntake(&intake, false, 0));

  // open climber
  climberButton
      .WhenPressed(frc2::SequentialCommandGroup(SetIntake(&intake, true, 0),
                                                frc2::WaitCommand(.2_s),
                                                SetClimber(&climber, true, 0)))
      .WhenReleased(frc2::SequentialCommandGroup(SetClimber(&climber, false, 0),
                                                 frc2::WaitCommand(.2_s),
                                                 SetIntake(&intake, false, 0)));

  climberMotorsTriggerRight
      .WhenActive(frc2::SequentialCommandGroup(SetClimber(&climber, false, 12),
                                               SetIntake(&intake, true, 0)))
      .WhenInactive(frc2::SequentialCommandGroup(SetClimber(&climber, false, 0),
                                                 SetIntake(&intake, false, 0)));

  climberMotorsTriggerLeft.WhenActive(SetClimber(&climber, false, -12))
      .WhenInactive(SetClimber(&climber, false, 0));
}

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
  // climber.reverseRightClimberMotor();
}

void Robot::TeleopPeriodic() {
  /*
std::cout << climber.getSwitch() << std::endl;

//Intake
  if (joystick.GetRawButton(5) == true){
    intake.intakeSolenoidForward();
    intake.initializeMotor();
  }else {
    intake.intakeSolenoidReverse();
    intake.desinitializeMotor();
    }

//Climber Up
  if (joystick.GetRawButton(4) == true){
    intake.intakeSolenoidForward();
    climber.climberSolenoidForward();
  } else {
    intake.intakeSolenoidReverse();
    climber.climberSolenoidReverse();
    }

  //Winches
  if (joystick.GetRawButton(1) && joystick.GetRawButton(6)){
    climber.initializeRightClimberMotor();
    climber.initializeLeftClimberMotor();
  } else {
    climber.desinitializeRightClimberMotor();
    climber.desinitializeLeftClimberMotor();
    }

 if (joystick.GetRawButton(1) && joystick.GetRawButton(5)){
    climber.reverseRightClimberMotor();
    climber.reverseLeftClimberMotor();
  } else {
    climber.desinitializeRightClimberMotor();
    climber.desinitializeLeftClimberMotor();
    }

  */
}

void Robot::DisabledInit() {}

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

// No fue facil, pero nos llevo al mundial y nos dejo en el ranking 29, se
// agradece infinitamente, gracias por compliar, gracias por avisarnos de
// errores, gracias por ser tan C++. Mulssane es un gran robot y su programa lo
// es tambien, excelente trabajo.
