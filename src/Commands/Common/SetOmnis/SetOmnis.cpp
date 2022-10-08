/*
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetOmnis.h"

SetOmnis::SetOmnis() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetOmnis::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetOmnis::Execute() {

  omnisMotor.Periodic();  
  
  //COSAS COMENTADAS SANTIQ 

  //std::int switchValue = ballCounter.GetName();
  //std::printf("%d\n", switchValue);
  
  
  
  //if (StorageAndDeliver::topLimit)

}

// Called once the command ends or is interrupted.
void SetOmnis::End(bool interrupted) {}

// Returns true when the command should end.
bool SetOmnis::IsFinished() {
  return false;
}
*/