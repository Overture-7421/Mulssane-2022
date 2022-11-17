#pragma once
#include <Subsystems/Intake/Intake.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include "Subsystems/Climber/Climber.h"

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
  frc2::Trigger intakeButton{[joystick = &joystick] {
    return joystick->GetRawButton(5) & !joystick->GetRawButton(1);
  }};
  frc2::JoystickButton climberButton{&joystick, 4};
  frc2::Trigger climberMotorsTriggerRight{[joystick = &joystick] {
    return joystick->GetRawButton(1) & joystick->GetRawButton(6);
  }};
  frc2::Trigger climberMotorsTriggerLeft{[joystick = &joystick] {
    return joystick->GetRawButton(1) & joystick->GetRawButton(5);
  }};
};
