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
  SwerveModule backRightModule{1, 2, 9, -143.70507812500001};
  SwerveModule backLeftModule{3, 4, 10, -70};
  SwerveModule frontLeftModule{5, 6, 11, -147.5};
  SwerveModule frontRightModule{7, 8, 12, -160};
  frc::Joystick joystick {0};
};
