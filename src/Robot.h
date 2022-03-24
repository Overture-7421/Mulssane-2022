/*
    ____                     __     ____        __          __     _   __
   /  _/___  ________  _____/ /_   / __ \____  / /_  ____  / /_   / | / /___
_____ ___  ___
   / // __ \/ ___/ _ \/ ___/ __/  / /_/ / __ \/ __ \/ __ \/ __/  /  |/ / __ `/
__ `__ \/ _ \
 _/ // / / (__  )  __/ /  / /_   / _, _/ /_/ / /_/ / /_/ / /_   / /|  / /_/ / /
/ / / /  __/
/___/_/ /_/____/\___/_/   \__/  /_/ |_|\____/_.___/\____/\__/  /_/ |_/\__,_/_/
/_/ /_/\___/
*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DutyCycleEncoder.h>

#include <frc2/command/Command.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>


#include "Commands/Autonomous/RamseteTests/RamseteTests.h"
#include "Commands/Autonomous/TurnToAngle/TurnToAngle.h"
#include "Commands/Teleop/DefaultDrive/DefaultDrive.h"
#include "Commands/Autonomous/Right_3BallAuto/Right_3BallAuto.h"
#include "Commands/Autonomous/Right_4BallAuto/Right_4BallAuto.h"
#include "Commands/Autonomous/Left_2BallAuto/Left_2BallAuto.h"


#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/Climber/Climber.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/RangeDecider/RangeDecider.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/StorageAndDeliver/StorageAndDeliver.h"
#include "Subsystems/VisionManager/VisionManager.h"
#include "Utils/Interpolation/LinearInterpolator/LinearInterpolator.h"

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
  frc::Joystick joy1{0}, joy2{1};

  // Subsystems
  Chassis chassis;
  Shooter shooter;
  Intake intake;
  StorageAndDeliver storageAndDeliver;
  Climber climber;
  VisionManager visionManager{&chassis};
  RangeDecider rangeDecider;

  DefaultDrive drive{&chassis, &visionManager, &rangeDecider, &joy1};
  frc2::JoystickButton intakeButton{&joy2, 5};
  frc2::JoystickButton feederShootButton{&joy2, 6};

  frc2::JoystickButton shootLongRangeButton{&joy2, 1};
  frc2::JoystickButton shootShortRangeButton{&joy2, 2};
  frc2::Trigger shootLowGoalButton {[joy2 = &joy2] { return joy2->GetRawAxis(3) > 0.5 && !joy2->GetRawButton(3); }};

  frc2::JoystickButton climberButtonUp{&joy2, 4};
  frc2::JoystickButton climberButtonMotorEnable{&joy2, 3};

   Left_2BallAuto left2BallAuto {&chassis, &visionManager, &intake, &storageAndDeliver, &shooter};
   Right_3BallAuto right3BallAuto {&chassis, &visionManager, &intake, &storageAndDeliver, &shooter};

   frc::SendableChooser<frc2::Command*> autoChooser;

   frc::DutyCycleEncoder absEnc {9};
};
