// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetHood.h"

SetHood::SetHood(Hood* hood, double setpoint) {
  this->hood = hood;
  this->setpoint = setpoint;
  AddRequirements(hood);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetHood::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetHood::Execute() {
    hood->SetHoodAngle(setpoint);
}

// Called once the command ends or is interrupted.
void SetHood::End(bool interrupted) {}

// Returns true when the command should end.
bool SetHood::IsFinished() {
  return true;
}
