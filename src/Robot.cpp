#include "Robot.h"
#include <frc2/command/CommandScheduler.h>
#include <iostream>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  frc::SmartDashboard::PutBoolean("buttonX : ",buttonX);
  frc::SmartDashboard::PutBoolean("buttonY : ",buttonY);
}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
  hood.negativeVoltageMotor();
}

void Robot::TeleopInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TeleopPeriodic() {
  std::cout << hood.getSwitch()<< std::endl;

  bool buttonX = PointShoot.GetXButton(); 

  bool buttonY = PointShoot.GetYButton();

  if (buttonX == true){
    hood.VoltageMotor();}
    else if (buttonY == true){
      hood.negativeVoltageMotor();}
      else {hood.noneVoltageMotor();}

  
  

  };

void Robot::DisabledInit() {
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






// No fue facil, pero nos llevo al mundial y nos dejo en el ranking 29, se agradece infinitamente, gracias por compliar, gracias por avisarnos de errores, gracias por ser tan C++. 
// Mulssane es un gran robot y su programa lo es tambien, excelente trabajo. 
