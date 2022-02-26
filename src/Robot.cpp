/*
    ____                     __     ____        __          __     _   __                   
   /  _/___  ________  _____/ /_   / __ \____  / /_  ____  / /_   / | / /___ _____ ___  ___ 
   / // __ \/ ___/ _ \/ ___/ __/  / /_/ / __ \/ __ \/ __ \/ __/  /  |/ / __ `/ __ `__ \/ _ \
 _/ // / / (__  )  __/ /  / /_   / _, _/ /_/ / /_/ / /_/ / /_   / /|  / /_/ / / / / / /  __/
/___/_/ /_/____/\___/_/   \__/  /_/ |_|\____/_.___/\____/\__/  /_/ |_/\__,_/_/ /_/ /_/\___/                                               
*/

#include "Robot.h"

#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include "frc2/command/CommandScheduler.h"

void Robot::RobotInit(){
  chassis.SetDefaultCommand(defaultDrive);
  chassis.resetOdometry(visionManager.getTargetPose());
}

void Robot::RobotPeriodic() {

  rangeDecider.updateRangeDecision(chassis.getPose(), visionManager.getTargetPose());
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {

  // autocommand = std::make_unique<RamseteTests>(&chassis);
  // autocommand->Schedule();
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
  intake.setVoltage((joy2.GetRawButton(5) ? 12 : 0));
  intake.setPistons(joy2.GetRawButton(5));

  storageAndDeliver.setFeederVoltage((joy2.GetRawButton(6) ? 12 : 0));
  storageAndDeliver.setIndexerVoltage((joy2.GetRawButton(6) ? 12 : 0));


  shooter.setVelocity(250);

  storageAndDeliver.setIndexerVoltage((joy2.GetRawButton(6) ? 12 : 0));

  bool currentA = joy2.GetRawButton(1);
  if(currentA != lastA && currentA){
    hoodState = !hoodState;
  }
  shooter.setHoodState(hoodState);

  lastA = currentA;
}

void Robot::DisabledInit() {
  
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
