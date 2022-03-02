// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "WaitBeReadyToShoot.h"

WaitBeReadyToShoot::WaitBeReadyToShoot(Shooter* shooter, VisionManager* visionManager) {
  this->shooter = shooter;
  this->visionManager = visionManager;
}

// Called when the command is initially scheduled.
void WaitBeReadyToShoot::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void WaitBeReadyToShoot::Execute() {}

// Called once the command ends or is interrupted.
void WaitBeReadyToShoot::End(bool interrupted) {}

// Returns true when the command should end.
bool WaitBeReadyToShoot::IsFinished() {
  return shooter->reachedVelocityTarget() && visionManager->isChassisAligned();
}
