// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "StorageAndDeliver.h"

#include <frc/smartdashboard/SmartDashboard.h>
StorageAndDeliver::StorageAndDeliver() {
  /* Upper Feeder Motor Configuration  */
  upperFeederMotor.SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20);
  upperFeederMotor.SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
    255);
  upperFeederMotor.ConfigOpenloopRamp(0.1);
  upperFeederMotor.ConfigSupplyCurrentLimit(
    SupplyCurrentLimitConfiguration(true, 15, 0, 1));
  upperFeederMotor.SetNeutralMode(NeutralMode::Brake);

  /* Lower Feeder Motor Configuration */
  lowerFeederMotor.SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20);
  lowerFeederMotor.SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
    255);
  lowerFeederMotor.ConfigOpenloopRamp(0.1);
  lowerFeederMotor.SetInverted(true);
  lowerFeederMotor.SetNeutralMode(NeutralMode::Brake);

  /* Omnis Motor Configuration */
  omnisMotor.SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20);
  omnisMotor.SetStatusFramePeriod(
    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
    255);

  omnisMotor.ConfigOpenloopRamp(0.1);
  omnisMotor.SetNeutralMode(NeutralMode::Brake);

  /* Counter Configuration */
  digitalGlitchFilter.SetPeriodNanoSeconds(1850000);
  ballCounter.SetUpSourceEdge(true, false);

#ifndef SIMULATION
  digitalGlitchFilter.Add(&ballCounter);
#endif
}

void StorageAndDeliver::setUpperFeederVoltage(double voltage) {
  upperFeederMotor.SetVoltage(units::volt_t(voltage));
}

void StorageAndDeliver::setLowerFeederVoltage(double voltage) {
  lowerFeederMotor.SetVoltage(units::volt_t(voltage));
}

void StorageAndDeliver::setOmnisMotorVoltage(double voltage) {
  omnisMotor.SetVoltage(units::volt_t(voltage));
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
