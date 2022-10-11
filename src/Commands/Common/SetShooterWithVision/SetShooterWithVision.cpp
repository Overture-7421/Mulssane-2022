// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetShooterWithVision.h"

SetShooterWithVision::SetShooterWithVision(Shooter* shooter, Hood* hood, VisionManager* visionManager) {
  this->shooter = shooter;
  this->hood = hood;
  this->visionManager = visionManager;
  AddRequirements({shooter, hood});
}

// Called when the command is initially scheduled.
void SetShooterWithVision::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void SetShooterWithVision::Execute() {
  const auto distance = visionManager->getDistanceToTarget().value();
  double shooterSetpoint = distanceVsVelocityInterpolator.getY(distance);
  shooter->setVelocity(shooterSetpoint);

  double hoodSetpoint = distanceVsAngleInterpolator.getY(distance);
  hood->SetHoodAngle(hoodSetpoint);
}

// Called once the command ends or is interrupted.
void SetShooterWithVision::End(bool interrupted) {}

bool SetShooterWithVision::IsFinished() {
  return true;
}
