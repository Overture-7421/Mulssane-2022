/*
    ____                     __     ____        __          __     _   __                   
   /  _/___  ________  _____/ /_   / __ \____  / /_  ____  / /_   / | / /___ _____ ___  ___ 
   / // __ \/ ___/ _ \/ ___/ __/  / /_/ / __ \/ __ \/ __ \/ __/  /  |/ / __ `/ __ `__ \/ _ \
 _/ // / / (__  )  __/ /  / /_   / _, _/ /_/ / /_/ / /_/ / /_   / /|  / /_/ / / / / / /  __/
/___/_/ /_/____/\___/_/   \__/  /_/ |_|\____/_.___/\____/\__/  /_/ |_/\__,_/_/ /_/ /_/\___/                                               
*/

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc2/command/Command.h>
#include "Subsystems/Chassis.h"
#include "Subsystems/Shooter.h"
#include "Autonomous/RamseteTests/RamseteTests.h"
#include "Teleop/DefaultDrive.h"
#include "Autonomous/TurnToAngle/TurnToAngle.h"

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
  std::unique_ptr<frc2::SequentialCommandGroup> autocommand;
  Shooter shooter;
  Chassis chassis;
  frc::Joystick joy {0};
  DefaultDrive defaultDrive {&chassis, &joy};
  TurnToAngle turnToAngle {&chassis, 0};

};
