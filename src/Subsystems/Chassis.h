// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>

class Chassis : public frc2::SubsystemBase {
 public:
  Chassis();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::motorcontrol::can::TalonFX chassisLeftMaster{1};
  ctre::phoenix::motorcontrol::can::TalonFX leftSlave_1{2}; 
  ctre::phoenix::motorcontrol::can::TalonFX leftSlave_2{3};
  ctre::phoenix::motorcontrol::can::TalonFX chassisRightMaster{11};
  ctre::phoenix::motorcontrol::can::TalonFX rightSlave_1{12}; 
  ctre::phoenix::motorcontrol::can::TalonFX rightSlave_2{13}; 
   
};
