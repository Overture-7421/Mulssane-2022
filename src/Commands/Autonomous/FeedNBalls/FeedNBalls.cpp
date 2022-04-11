// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "FeedNBalls.h"

FeedNBalls::FeedNBalls(StorageAndDeliver* storageAndDeliver, int ballsToShoot) {
  this->storageAndDeliver = storageAndDeliver;
  this->ballsToShoot = ballsToShoot;
  AddRequirements(storageAndDeliver);
}

// Called when the command is initially scheduled.
void FeedNBalls::Initialize() {
  initialCount = storageAndDeliver->getBallsShot();
}

// Called repeatedly when this Command is scheduled to run
void FeedNBalls::Execute() {
  storageAndDeliver->setFeederVoltage(8);
  storageAndDeliver->setIndexerVoltage(8);
}

// Called once the command ends or is interrupted.
void FeedNBalls::End(bool interrupted) {}

// Returns true when the command should end.
bool FeedNBalls::IsFinished() {
  int ballsShot = storageAndDeliver->getBallsShot() - initialCount;
  return ballsShot >= ballsToShoot;
}
