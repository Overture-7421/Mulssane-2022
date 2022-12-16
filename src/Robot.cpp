#include "Robot.h"
#include <frc2/command/CommandScheduler.h>
#ifndef RUNNING_FRC_TESTS
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();


}

void Robot::AutonomousInit() {
  swervePath.Schedule();
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
  swervePath.Cancel();
}

void Robot::TeleopPeriodic() {
  frc::ChassisSpeeds chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    units::meters_per_second_t{ -joystick.GetRawAxis(1) * 5 },
    units::meters_per_second_t{ -joystick.GetRawAxis(0) * 5 },
    units::radians_per_second_t(-joystick.GetRawAxis(4) * 9),
    swerveChassis.getOdometry().Rotation());

  swerveChassis.setSpeed(chassisSpeeds.vx.value(), chassisSpeeds.vy.value(), chassisSpeeds.omega.value());

}


void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

int main() {
  // These warnings generate console prints that cause scheduling jitter
  frc::DriverStation::SilenceJoystickConnectionWarning(true);
  // This telemetry regularly causes loop overruns
  frc::LiveWindow::DisableAllTelemetry();

  return frc::StartRobot<Robot>();
}
#endif

// No fue facil, pero nos llevo al mundial y nos dejo en el ranking 29, se agradece infinitamente, gracias por compliar, gracias por avisarnos de errores, gracias por ser tan C++.
// Mulssane es un gran robot y su programa lo es tambien, excelente trabajo. sus
