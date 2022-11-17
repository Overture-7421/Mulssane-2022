#pragma once

#include <frc/TimedRobot.h>
#include <Subsystems/SwerveModule.h>
#include <frc/Joystick.h>


class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override; 

 

  private:
  SwerveModule backRightModule{1, 2, 9, 37.70507812500001};
  SwerveModule backLeftModule{3, 4, 10, 0};
  SwerveModule frontRightModule{5, 6, 11, 0};
  SwerveModule frontLeftModule{7, 8, 12, 0};
  frc::Joystick joystick {0};
};
