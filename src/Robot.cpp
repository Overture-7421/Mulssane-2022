#include "Robot.h"
#include <frc2/command/CommandScheduler.h>
#ifndef RUNNING_FRC_TESTS
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{ 
}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit()
{
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit()
{
  frc2::CommandScheduler::GetInstance().CancelAll();

  frc::SmartDashboard::PutNumber("Left Kp", 0);
  frc::SmartDashboard::PutNumber("Right Kp", 0);
  frc::SmartDashboard::PutNumber("Left Kp", 0);
  frc::SmartDashboard::PutNumber("Left Ki", 0);
  frc::SmartDashboard::PutNumber("Right Ki", 0);
  frc::SmartDashboard::PutNumber("Left Kd", 0);
  frc::SmartDashboard::PutNumber("Right Kd", 0);

  frc::SmartDashboard::PutNumber("Left f", 0);
  frc::SmartDashboard::PutNumber("Right f", 0);

}

void Robot::TeleopPeriodic()
{
  chassis.setSpeed(-joystick.GetRawAxis(1) * 3, -joystick.GetRawAxis(4) * 2 * M_PI);

  frc::SmartDashboard::PutNumber("LeftSpeed", chassis.getLeftSpeed());
  frc::SmartDashboard::PutNumber("RightSpeed", chassis.getRightSpeed());

  double leftKp = frc::SmartDashboard::GetNumber("Left Kp", 0);
  double rightKp = frc::SmartDashboard::GetNumber("Right Kp", 0);
  double leftKi = frc::SmartDashboard::GetNumber("Left Ki", 0);
  double rightKi = frc::SmartDashboard::GetNumber("Right Ki", 0);
  double leftKd = frc::SmartDashboard::GetNumber("Left Kd", 0);
  double rightKd = frc::SmartDashboard::GetNumber("Right Kd", 0);

  double leftF = frc::SmartDashboard::GetNumber("Left f", 0);
  double rightF = frc::SmartDashboard::GetNumber("Right f", 0);
/*
  chassis.setPIDvaluesLeft(leftKp, leftKi, leftKd, leftF);

  chassis.setPIDvaluesRight(rightKp, rightKi, rightKd, rightF);
*/
}


void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

int main()
{
  // These warnings generate console prints that cause scheduling jitter
  frc::DriverStation::SilenceJoystickConnectionWarning(true);
  // This telemetry regularly causes loop overruns
  frc::LiveWindow::DisableAllTelemetry();

  return frc::StartRobot<Robot>();
}
#endif

// No fue facil, pero nos llevo al mundial y nos dejo en el ranking 29, se agradece infinitamente, gracias por compliar, gracias por avisarnos de errores, gracias por ser tan C++.
// Mulssane es un gran robot y su programa lo es tambien, excelente trabajo. sus
