// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  TalonSRX intakeMotor {9};
  frc::DoubleSolenoid intakeSolenoid {frc::PneumaticsModuleType::CTREPCM, 1, 2};

//maybe in commands >> common
//intakeSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
//exampleDoublePCM.Set(frc::DoubleSolenoid::Value::kForward);
//exampleDoublePCM.Set(frc::DoubleSolenoid::Value::kReverse);

};
