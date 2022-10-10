// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Omnis.h"

Omnis::Omnis() {
    omnisMotor.SetStatusFramePeriod(
        ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20);
    omnisMotor.SetStatusFramePeriod(
        ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
        255);

    omnisMotor.ConfigOpenloopRamp(0.1);
}

void Omnis::setVoltage(double voltage) {
    omnisMotor.SetVoltage(units::volt_t(voltage));
    // -8_V
}

// This method will be called once per scheduler run
void Omnis::Periodic() {
}