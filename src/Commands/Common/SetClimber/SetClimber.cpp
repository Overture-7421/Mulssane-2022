// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetClimber.h"

SetClimber::SetClimber(Climber* climber, bool pistonState, double voltage) {
  this->pistonState = pistonState;
  this->voltage = voltage;
  this->climber = climber;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(climber);
}

// Called when the command is initially scheduled.
void SetClimber::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetClimber::Execute() {
  climber->setPistons(pistonState);
  climber->setMotor(voltage);
}

// Called once the command ends or is interrupted.
void SetClimber::End(bool interrupted) {}

// Returns true when the command should end.
bool SetClimber::IsFinished() { return true; }
