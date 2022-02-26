// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Climber.h"

Climber::Climber(){
    leftClimber.ConfigOpenloopRamp(0.1);
    rightClimber.ConfigOpenloopRamp(0.1);

    
    setPistons(false);

}

void Climber::setPistons(bool set){
    climberPiston.Set(set);
}

void Climber::setVoltage(double voltage){
    leftClimber.SetVoltage(units::volt_t(voltage));
    rightClimber.SetVoltage(units::volt_t(voltage));
}


// This method will be called once per scheduler run
void Climber::Periodic() {}
