// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetShooter.h"

SetShooter::SetShooter(Shooter* shooter, double setpoint, bool hoodState) {
  this->shooter = shooter;
  this->setpoint = setpoint;
  this->hoodState = hoodState;
  AddRequirements(shooter);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetShooter::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetShooter::Execute() {
  shooter->setHoodState(hoodState);
  shooter->setVelocity(setpoint);
}

// Called once the command ends or is interrupted.
void SetShooter::End(bool interrupted) {}

// Returns true when the command should end.
bool SetShooter::IsFinished() {
  return true;
}
