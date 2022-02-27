// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Climber.h"

#include <frc/smartdashboard/SmartDashboard.h>
Climber::Climber() {
  leftClimber.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20);
  leftClimber.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
      255);

  rightClimber.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20);
  rightClimber.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
      255);

  leftClimber.ConfigOpenloopRamp(0.1);
  rightClimber.ConfigOpenloopRamp(0.1);

  leftClimber.SetNeutralMode(NeutralMode::Brake);
  rightClimber.SetNeutralMode(NeutralMode::Brake);

  setPistons(false);
}

void Climber::setPistons(bool set) { climberPiston.Set(set); }

void Climber::setVoltage(double voltage) {
  leftClimber.SetVoltage(units::volt_t(voltage));
  rightClimber.SetVoltage(units::volt_t(voltage));
}

// This method will be called once per scheduler run
void Climber::Periodic() {}
