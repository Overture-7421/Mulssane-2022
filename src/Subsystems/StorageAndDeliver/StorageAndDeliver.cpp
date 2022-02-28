// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "StorageAndDeliver.h"

#include <frc/smartdashboard/SmartDashboard.h>
StorageAndDeliver::StorageAndDeliver() {
  indexerMotor.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20);
  indexerMotor.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
      255);

  upperFeederMotor.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20);
  upperFeederMotor.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
      255);

  lowerFeederMotor.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20);
  lowerFeederMotor.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
      255);

  indexerMotor.ConfigOpenloopRamp(0.1);
  upperFeederMotor.ConfigOpenloopRamp(0.1);
  lowerFeederMotor.ConfigOpenloopRamp(0.1);

  indexerMotor.SetInverted(true);
  lowerFeederMotor.SetInverted(true);
  indexerMotor.ConfigSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration(true, 15, 0, 1));
  upperFeederMotor.ConfigSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration(true, 15, 0, 1));

  indexerMotor.SetNeutralMode(NeutralMode::Brake);
  upperFeederMotor.SetNeutralMode(NeutralMode::Brake);
  lowerFeederMotor.SetNeutralMode(NeutralMode::Brake);

  digitalGlitchFilter.SetPeriodNanoSeconds(1650000);

#ifndef SIMULATION
  digitalGlitchFilter.Add(&ballCounter);
#endif
}

void StorageAndDeliver::setIndexerVoltage(double voltage) {
  indexerMotor.SetVoltage(units::volt_t(voltage));
}

void StorageAndDeliver::setFeederVoltage(double voltage) {
  upperFeederMotor.SetVoltage(units::volt_t(voltage));
  lowerFeederMotor.SetVoltage(units::volt_t(voltage));
}

int StorageAndDeliver::getBallsShot() { return ballCounter.Get(); }

bool StorageAndDeliver::isTopSwitchPressed() { return !topLimit.Get(); }

bool StorageAndDeliver::isBottomSwitchPressed() { return !bottomLimit.Get(); }

// This method will be called once per scheduler run
void StorageAndDeliver::Periodic() {
  frc::SmartDashboard::PutNumber("StorageAndDeliver/BallsShot", getBallsShot());
  frc::SmartDashboard::PutBoolean("StorageAndDeliver/TopLimit",
                                  isTopSwitchPressed());
  frc::SmartDashboard::PutBoolean("StorageAndDeliver/BottomLimit",
                                  isBottomSwitchPressed());
}
