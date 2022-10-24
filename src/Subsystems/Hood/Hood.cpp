// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Hood.h"
#include <frc/smartdashboard/SmartDashboard.h>

Hood::Hood() {
    hoodMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);
    hoodMotor.SetSensorPhase(true);
}
// This method will be called once per scheduler run
void Hood::Periodic() {
    switch (operationState)
    {
        case OperationState::Homing:
        {
            MoveToLimit();
            if(GetSwitch()){
                operationState = OperationState::PID;
                hoodMotor.SetSelectedSensorPosition(0.0);
                positionController.Reset(units::scalar_t(0.0));
            }
            break;
        }

        case OperationState::PID:
        {
            positionController.SetGoal(units::scalar_t(hoodTarget));
            double output = positionController.Calculate(units::scalar_t(GetHoodAngle()));
            frc::SmartDashboard::PutNumber("Hood/Output", output);
            hoodMotor.SetVoltage(units::volt_t(output));
            break;
        }
        default:
            break;
    }

    frc::SmartDashboard::PutNumber("Hood/HoodAngle", GetHoodAngle());
    frc::SmartDashboard::PutNumber("Hood/OperationState", operationState == OperationState::Homing ? 0 : 1);
}
