// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <units/voltage.h> 
#include <frc/Encoder.h>
#include <frc/controller/ProfiledPIDController.h>
#include "OperationState.h"

class Hood : public frc2::SubsystemBase {
 public:
  Hood();

  void Periodic() override;

  void MoveMotor(units::voltage::volt_t voltage){
    hoodMotor.SetVoltage(voltage);

  }

  bool GetSwitch() {
    return !limitSwitch.Get();
  }

  void MoveToLimit(){
    if(GetSwitch()){
      MoveMotor(0_V);
    }else{
      MoveMotor(-3_V);
    }
  }

  double GetHoodAngle(){
    return hoodMotor.GetSelectedSensorPosition() / upperLimit;
  }
  //set entre 0 y 1
  void SetHoodAngle(double set){
    hoodTarget = std::clamp(set, 0.0, 1.0);
  }
  
 private:
  WPI_TalonSRX hoodMotor{9};
  frc::DigitalInput limitSwitch {2};
  frc::ProfiledPIDController<units::scalar> positionController {25.0, 5.0, 0.0, {units::scalar_t(3) / units::second_t(1), units::scalar_t(2) / (units::second_t(1) * units::second_t(1))}};
  OperationState operationState = OperationState::Homing;
  const double lowerLimit = 0;
  const double upperLimit = 9080;
  double hoodTarget = 0;
};
