// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetClimberVoltage.h"

SetClimberVoltage::SetClimberVoltage(Climber* climber, double voltage) {
  this->voltage = voltage;
  this->climber = climber;
  AddRequirements(climber);
}

// Called when the command is initially scheduled.
void SetClimberVoltage::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetClimberVoltage::Execute() {
  climber->setVoltage(voltage);
}

// Called once the command ends or is interrupted.
void SetClimberVoltage::End(bool interrupted) {}

// Returns true when the command should end.
bool SetClimberVoltage::IsFinished() {
  return true;
}
