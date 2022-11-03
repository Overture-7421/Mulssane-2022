#include "Robot.h"
#include <frc2/command/CommandScheduler.h>
#ifndef RUNNING_FRC_TESTS
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>

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
}

void Robot::TeleopPeriodic()
{
  chassis.setSpeed(-joystick.GetRawAxis(1) * 8, -joystick.GetRawAxis(4) * 2 * M_PI);
  
  chassis.getEncoder();
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
