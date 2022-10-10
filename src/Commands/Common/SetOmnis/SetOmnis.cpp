
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetOmnis.h"

SetOmnis::SetOmnis(Omnis* omnisMotor, double voltage) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->omnisMotor = omnisMotor;
  this->voltage = voltage;
}

// Called when the command is initially scheduled.
void SetOmnis::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetOmnis::Execute() {
  omnisMotor->setVoltage(voltage);
}

// Called once the command ends or is interrupted.
void SetOmnis::End(bool interrupted) {}

// Returns true when the command should end.
bool SetOmnis::IsFinished() {
  return true;
}