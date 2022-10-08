/*
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Omnis/Omnis.h>
#include <subsystems/StorageAndDeliver/StorageAndDeliver.h>



/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 
class SetOmnis
    : public frc2::CommandHelper<frc2::CommandBase, SetOmnis> {
 public:
  SetOmnis();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:

  Omnis omnisMotor;
  frc::DigitalInput topLimit {1};
  frc::Counter ballCounter {&topLimit};

};
*/
