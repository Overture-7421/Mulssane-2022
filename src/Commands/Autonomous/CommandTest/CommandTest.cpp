// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <iostream>
#include "CommandTest.h"

CommandTest::CommandTest(Chassis* chassis, std::string name1, std::string name2) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(chassis);
}

// Called when the command is initially scheduled.
void CommandTest::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CommandTest::Execute() { 

  std::cout<< name1 + name2 <<std::endl;
}

// Called once the command ends or is interrupted.
void CommandTest::End(bool interrupted) {}

// Returns true when the command should end.
bool CommandTest::IsFinished() {
  return false;
}
