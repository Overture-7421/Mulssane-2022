// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AutoPreloadBall.h"
#include <iostream>
AutoPreloadBall::AutoPreloadBall(StorageAndDeliver* storageAndDeliver) {
  this->storageAndDeliver = storageAndDeliver;
  AddRequirements(storageAndDeliver);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoPreloadBall::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void AutoPreloadBall::Execute() {
  if(!storageAndDeliver->isTopSwitchPressed()){
    storageAndDeliver->setUpperFeederVoltage(3.5);
  }else{
    storageAndDeliver->setUpperFeederVoltage(0);
  }

  if(storageAndDeliver->isBottomSwitchPressed() && storageAndDeliver->isTopSwitchPressed()){
    storageAndDeliver->setLowerFeederVoltage(0);
    storageAndDeliver->setOmnisMotorVoltage(0);
  }else{
    storageAndDeliver->setLowerFeederVoltage(2);
    storageAndDeliver->setOmnisMotorVoltage(12);
  }
}

// Called once the command ends or is interrupted.
void AutoPreloadBall::End(bool interrupted) {

}

// Returns true when the command should end.
bool AutoPreloadBall::IsFinished() {
  return (storageAndDeliver->isTopSwitchPressed() && storageAndDeliver->isBottomSwitchPressed());
}