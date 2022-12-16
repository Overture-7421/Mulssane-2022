#pragma once
#include "Subsystems/SwerveChassis/SwerveChassis.h"
#include "Commands/Common/FollowSwervePath/FollowSwervePath.h"

#include <frc/TimedRobot.h>
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
  SwerveChassis swerveChassis;
  frc::Joystick joystick{ 0 };
  FollowSwervePath swervePath{ &swerveChassis, {
    {0_m,0_m,{0_deg}},
    {7_m,0_m,{0_deg}},
    {7_m,1.5_m,{90_deg}},
    {0_m,1.5_m,{180_deg}},
    {0_m,0_m,{180_deg}}
    } };
};
