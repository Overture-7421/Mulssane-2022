// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetStorageAndDeliver.h"

SetStorageAndDeliver::SetStorageAndDeliver(StorageAndDeliver* storageAndDeliver,
                                           double voltage) {
  this->storageAndDeliver = storageAndDeliver;
  this->voltage = voltage;
  AddRequirements(storageAndDeliver);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetStorageAndDeliver::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetStorageAndDeliver::Execute() {
  storageAndDeliver->setFeederVoltage(voltage);
  storageAndDeliver->setIndexerVoltage(voltage);
}

// Called once the command ends or is interrupted.
void SetStorageAndDeliver::End(bool interrupted) {}

// Returns true when the command should end.
bool SetStorageAndDeliver::IsFinished() { return true; }
