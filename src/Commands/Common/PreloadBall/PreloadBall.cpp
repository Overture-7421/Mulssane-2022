// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PreloadBall.h"

PreloadBall::PreloadBall(StorageAndDeliver* storageAndDeliver,
                         bool useAsDefault) {
  this->storageAndDeliver = storageAndDeliver;
  this->useAsDefault = useAsDefault;
  AddRequirements(storageAndDeliver);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PreloadBall::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PreloadBall::Execute() {
  if (!storageAndDeliver->isTopSwitchPressed()) {
    storageAndDeliver->setFeederVoltage(6);
    storageAndDeliver->setIndexerVoltage(6);
  } else {
    storageAndDeliver->setIndexerVoltage(0);
    storageAndDeliver->setFeederVoltage(0);
  }


}

// Called once the command ends or is interrupted.
void PreloadBall::End(bool interrupted) {
  storageAndDeliver->setFeederVoltage(0);
  storageAndDeliver->setIndexerVoltage(0);
}

// Returns true when the command should end.
bool PreloadBall::IsFinished() {
  if (useAsDefault) {
    return false;
  }

  return storageAndDeliver->isTopSwitchPressed();
}
