// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

class Chassis : public frc2::SubsystemBase {
 public:
  Chassis();

  void moveForward() {
    chassisLeftMaster.SetVoltage(2_V);
    chassisRightMaster.SetVoltage(-2_V);
    leftSlave_1.SetVoltage(2_V);
    rightSlave_1.SetVoltage(-2_V);
    leftSlave_2.SetVoltage(2_V);
    rightSlave_2.SetVoltage(-2_V);
  }

  void moveBackward() {
    chassisLeftMaster.SetVoltage(-2_V);
    chassisRightMaster.SetVoltage(2_V);
    leftSlave_1.SetVoltage(-2_V);
    rightSlave_1.SetVoltage(2_V);
    leftSlave_2.SetVoltage(-2_V);
    rightSlave_2.SetVoltage(2_V);
  }

  void moveLeft() {
    chassisLeftMaster.SetVoltage(1_V);
    chassisRightMaster.SetVoltage(-2_V);
    leftSlave_1.SetVoltage(1_V);
    rightSlave_1.SetVoltage(-2_V);
    leftSlave_2.SetVoltage(1_V);
    rightSlave_2.SetVoltage(-2_V);
  }

  void moveRight() {
    chassisLeftMaster.SetVoltage(4_V);
    chassisRightMaster.SetVoltage(-2_V);
    leftSlave_1.SetVoltage(4_V);
    rightSlave_1.SetVoltage(-2_V);
    leftSlave_2.SetVoltage(4_V);
    rightSlave_2.SetVoltage(-2_V);
  }

  void stopMovement() {
    chassisLeftMaster.SetVoltage(0_V);
    chassisRightMaster.SetVoltage(0_V);
    leftSlave_1.SetVoltage(0_V);
    rightSlave_1.SetVoltage(0_V);
    leftSlave_2.SetVoltage(0_V);
    rightSlave_2.SetVoltage(0_V);
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;



 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::motorcontrol::can::WPI_TalonFX chassisLeftMaster{1};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX leftSlave_1{2}; 
  ctre::phoenix::motorcontrol::can::WPI_TalonFX leftSlave_2{3};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX chassisRightMaster{11};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX rightSlave_1{12}; 
  ctre::phoenix::motorcontrol::can::WPI_TalonFX rightSlave_2{13}; 
   
};
