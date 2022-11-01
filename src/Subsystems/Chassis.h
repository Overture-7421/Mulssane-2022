// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>

class Chassis : public frc2::SubsystemBase {
 public:
  Chassis();

  void moveForward(double y) {
    chassisLeftMaster.SetVoltage(units::volt_t(-y));
    chassisRightMaster.SetVoltage(units::volt_t(y));
    leftSlave_1.SetVoltage(units::volt_t(-y));
    rightSlave_1.SetVoltage(units::volt_t(y));
    leftSlave_2.SetVoltage(units::volt_t(-y));
    rightSlave_2.SetVoltage(units::volt_t(y));
  }

  void moveLeft(double x) {
    chassisLeftMaster.SetVoltage(units::volt_t(x/4));
    chassisRightMaster.SetVoltage(units::volt_t(x));
    leftSlave_1.SetVoltage(units::volt_t(x/4));
    rightSlave_1.SetVoltage(units::volt_t(x));
    leftSlave_2.SetVoltage(units::volt_t(x/4));
    rightSlave_2.SetVoltage(units::volt_t(x));
  }

  void moveRight(double x) {
    chassisLeftMaster.SetVoltage(units::volt_t(x));
    chassisRightMaster.SetVoltage(units::volt_t(-x/4));
    leftSlave_1.SetVoltage(units::volt_t(x));
    rightSlave_1.SetVoltage(units::volt_t(-x/4));
    leftSlave_2.SetVoltage(units::volt_t(x));
    rightSlave_2.SetVoltage(units::volt_t(-x/4));
  }


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;



 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX chassisLeftMaster{1};
  WPI_TalonFX leftSlave_1{2}; 
  WPI_TalonFX leftSlave_2{3};
  WPI_TalonFX chassisRightMaster{11};
  WPI_TalonFX rightSlave_1{12}; 
  WPI_TalonFX rightSlave_2{13}; 
   
};
