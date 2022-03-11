// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetShooterWithVision.h"

SetShooterWithVision::SetShooterWithVision(Shooter* shooter, VisionManager* visionManager) {
  this->shooter = shooter;
  this->visionManager = visionManager;
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void SetShooterWithVision::Initialize() {
  shooter->setHoodState(true);
}

// Called repeatedly when this Command is scheduled to run
void SetShooterWithVision::Execute() {
  const auto distance = visionManager->getDistanceToTarget().value();
  double shooterSetpoint = distanceVsVelocityInterpolator.getY(distance) + 9;
  shooter->setVelocity(shooterSetpoint);
}

// Called once the command ends or is interrupted.
void SetShooterWithVision::End(bool interrupted) {}

bool SetShooterWithVision::IsFinished() {
  return true;
}
