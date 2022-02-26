// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake(){
    intakeMotor.ConfigOpenloopRamp(0.1);
    intakeMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 10, 0, 1));
}

void Intake::setPistons(bool set) {
  if (set) {
    intakePiston.Set(frc::DoubleSolenoid::kForward);
  } else {
    intakePiston.Set(frc::DoubleSolenoid::kReverse);
  }
}

void Intake::setVoltage(double voltage){
    intakeMotor.SetVoltage(units::volt_t(voltage));
}

// This method will be called once per scheduler run
void Intake::Periodic() {}
