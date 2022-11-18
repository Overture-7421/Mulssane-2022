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
  frc::SmartDashboard::PutNumber("backLeftModule", backLeftModule.returnPosition());
  frc::SmartDashboard::PutNumber("backRightModule", backRightModule.returnPosition());
  frc::SmartDashboard::PutNumber("frontLeftModule", frontLeftModule.returnPosition());
  frc::SmartDashboard::PutNumber("frontRightModule", frontRightModule.returnPosition());


  backRightModule.Periodic();
  backLeftModule.Periodic();
  frontRightModule.Periodic();
  frontLeftModule.Periodic();
}

void Robot::AutonomousInit()
{
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit()
{
  frc::SmartDashboard::PutNumber("Kp", 0.125);
  frc::SmartDashboard::PutNumber("Ki", 0.5);
  frc::SmartDashboard::PutNumber("Kd", 0);
  frc::SmartDashboard::PutNumber("f", 0);


 frc::SmartDashboard::PutNumber("TargetAngle", 0.0);
}

void Robot::TeleopPeriodic()
{ 
  double kP = frc::SmartDashboard::GetNumber("Kp", 0);
  double kI = frc::SmartDashboard::GetNumber("Ki", 0);
  double kD = frc::SmartDashboard::GetNumber("Kd", 0);
  double f = frc::SmartDashboard::GetNumber("f", 0);

  //swerve.setPIDvalues(kP, kI, kD, f);
  double targetAngle = joystick.GetDirectionDegrees() ;
  //double targetAngle = frc::SmartDashboard::GetNumber("TargetAngle", 0.0);
  backRightModule.setAngle(targetAngle);
  backLeftModule.setAngle(targetAngle);
  frontRightModule.setAngle(targetAngle);
  frontLeftModule.setAngle(targetAngle);

  backRightModule.SetWheelVoltage(-joystick.GetRawAxis(5) * 3);
  backLeftModule.SetWheelVoltage(-joystick.GetRawAxis(5) * 3);
  frontRightModule.SetWheelVoltage(-joystick.GetRawAxis(5) * 3);
  frontLeftModule.SetWheelVoltage(-joystick.GetRawAxis(5) * 3);
   
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
