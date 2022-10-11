/*
    __  _____  ____   __________ ___    _   ________   ____        __          __     ______          __
   /  |/  / / / / /  / ___/ ___//   |  / | / / ____/  / __ \____  / /_  ____  / /_   / ____/___  ____/ /__
  / /|_/ / / / / /   \__ \\__ \/ /| | /  |/ / __/    / /_/ / __ \/ __ \/ __ \/ __/  / /   / __ \/ __  / _ \
 / /  / / /_/ / /______/ /__/ / ___ |/ /|  / /___   / _, _/ /_/ / /_/ / /_/ / /_   / /___/ /_/ / /_/ /  __/
/_/  /_/\____/_____/____/____/_/  |_/_/ |_/_____/  /_/ |_|\____/_.___/\____/\__/   \____/\____/\__,_/\___/

*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/TimedRobot.h>
#include <frc/AddressableLed.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include "Commands/Autonomous/RamseteTests/RamseteTests.h"
#include "Commands/Autonomous/TurnToAngle/TurnToAngle.h"
#include "Commands/Teleop/DefaultDrive/DefaultDrive.h"
#include "Commands/Autonomous/Right_3BallAuto/Right_3BallAuto.h"
#include "Commands/Autonomous/Right_4BallAuto/Right_4BallAuto.h"
#include "Commands/Autonomous/Left_2BallAuto/Left_2BallAuto.h"
#include "Commands/Autonomous/Center_SingleBallAuto/Center_SingleBallAuto.h"
#include "Commands/Autonomous/Left_Kidnap/Left_Kidnap.h"

#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/Climber/Climber.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/StorageAndDeliver/StorageAndDeliver.h"
#include "Subsystems/VisionManager/VisionManager.h"
#include "Subsystems/Hood/Hood.h"
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
  static constexpr int kLength = 60;

  /* Subsystems */
  Chassis chassis;
  Shooter shooter;
  Intake intake;
  StorageAndDeliver storageAndDeliver;
  Hood hood;
  Climber climber;
  VisionManager visionManager{ &chassis };
  DefaultDrive drive{ &chassis, &visionManager, &joy1 };

  /* Buttons */
  frc::Joystick joy1{ 0 }, joy2{ 1 };
  frc2::JoystickButton intakeButton{ &joy2, 5 };
  frc2::JoystickButton feederShootButton{ &joy2, 6 };
  frc2::JoystickButton shootWithVisionButton{ &joy2, 1 };
  //   frc2::JoystickButton shootShortRangeButton{&joy2, 2};
  //   frc2::Trigger shootLowGoalButton {[joy2 = &joy2] { return joy2->GetRawAxis(3) > 0.5 && !joy2->GetRawButton(3); }};
  frc2::Trigger spitBallsTrigger{ [joy2 = &joy2] { return joy2->GetRawAxis(2) > 0.5 && !joy2->GetRawButton(3); } };
  frc2::JoystickButton climberButtonUp{ &joy2, 4 };
  frc2::JoystickButton climberButtonMotorEnable{ &joy2, 3 };

  /* Autonomos */
  //    Left_2BallAuto left2BallAuto {&chassis, &visionManager, &intake, &storageAndDeliver, &shooter};
  //    Right_3BallAuto right3BallAuto {&chassis, &visionManager, &intake, &storageAndDeliver, &shooter};
  //    Center_SingleBallAuto centerSingleBallAuto {&chassis, &visionManager, &intake, &storageAndDeliver, &shooter};
  //    Left_Kidnap leftKidnap {&chassis, &visionManager, &intake, &storageAndDeliver, &shooter};

  //    frc::SendableChooser<frc2::Command*> autoChooser;

};
