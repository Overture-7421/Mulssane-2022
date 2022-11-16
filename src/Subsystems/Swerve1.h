// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>

class Swerve1 : public frc2::SubsystemBase {
 public:
  Swerve1(){
    canCoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  }
  
  void SetVoltage(double rotatorVoltage, double wheelVoltage) {
    rotator.SetVoltage(units::volt_t(rotatorVoltage));
    wheel.SetVoltage(units::volt_t(wheelVoltage));
  }

  double returnPosition(){
    return canCoder.GetAbsolutePosition();
  }  

  double getPID(double rotationPID){
    return rotationPID
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override {

  }

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX rotator{1};
  WPI_TalonFX wheel{2};

  CANCoder canCoder{9};
  frc2::PIDController rotatorPID{0, 0, 0};
};
