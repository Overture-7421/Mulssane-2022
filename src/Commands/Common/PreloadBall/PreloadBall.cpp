// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PreloadBall.h"
#include <iostream>
PreloadBall::PreloadBall(StorageAndDeliver* storageAndDeliver, Omnis* omnis) {
  this->storageAndDeliver = storageAndDeliver;
  this->omnis = omnis;
  AddRequirements(storageAndDeliver);
  AddRequirements(omnis);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PreloadBall::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void PreloadBall::Execute() {
  if (storageAndDeliver->isBottomSwitchPressed() && storageAndDeliver->isTopSwitchPressed()) {
    storageAndDeliver->setLowerFeederVoltage(0);
    storageAndDeliver->setUpperFeederVoltage(0);
    omnis->setVoltage(0);
  } else if (storageAndDeliver->isTopSwitchPressed()) {
    storageAndDeliver->setUpperFeederVoltage(0);
  }
}

// Called once the command ends or is interrupted.
void PreloadBall::End(bool interrupted) {

}

// Returns true when the command should end.
bool PreloadBall::IsFinished() {
  return (storageAndDeliver->isTopSwitchPressed() && storageAndDeliver->isBottomSwitchPressed());
}
