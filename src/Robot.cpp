#include "Robot.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include <iostream>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() 
{
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TeleopPeriodic() {

  if (joystick.GetRawButton(5) == true){
    intake.initializeMotor();
    intake.intakeSolenoidForward();
  }else {
    intake.desinitializeMotor();
    intake.intakeSolenoidReverse();
    }

  if (joystick.GetRawButton(4) == true){
    intake.intakeSolenoidForward();
    climber.climberSolenoidForward();
  } else {
    intake.intakeSolenoidReverse();
    climber.climberSolenoidReverse();
    }
  
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
