#pragma once
#include <frc/TimedRobot.h>
#include <Subsystems/Intake/Intake.h>
#include "Subsystems/Climber/Climber.h"
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>

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
  Intake intake;
  Climber climber;
  frc::Joystick joystick{0};
  frc2::JoystickButton intakeButton{&joystick, 5};
  frc2::JoystickButton climberButton{&joystick, 4};

  
};
