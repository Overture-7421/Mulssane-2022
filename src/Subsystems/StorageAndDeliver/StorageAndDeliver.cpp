// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "StorageAndDeliver.h"

StorageAndDeliver::StorageAndDeliver(){
    indexerMotor.ConfigOpenloopRamp(0.1);
    upperFeederMotor.ConfigOpenloopRamp(0.1);
    lowerFeederMotor.ConfigOpenloopRamp(0.1);

    indexerMotor.SetInverted(true);
    upperFeederMotor.SetInverted(true);
    indexerMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 10, 0, 1));
    upperFeederMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 20, 0, 1));

}

void StorageAndDeliver::setIndexerVoltage(double voltage){
    indexerMotor.SetVoltage(units::volt_t(voltage));
}

void StorageAndDeliver::setFeederVoltage(double voltage){
    upperFeederMotor.SetVoltage(units::volt_t(-voltage));
    lowerFeederMotor.SetVoltage(units::volt_t(voltage));
}

// This method will be called once per scheduler run
void StorageAndDeliver::Periodic() {}
