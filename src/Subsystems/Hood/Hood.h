// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <units/units.h>

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

 private:
  WPI_TalonSRX hoodMotor{9};
  frc::DigitalInput limitSwitch {2};
};
