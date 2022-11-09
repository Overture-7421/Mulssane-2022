// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>

class Climber : public frc2::SubsystemBase {
 public:
  Climber();
  void invertClimberMotors() {
    rightClimberMotor.SetInverted(true);
    leftClimberMotor.SetInverted(true);
  };

  void initializeRightClimberMotor() {
    rightClimberMotor.Set(VictorSPXControlMode::PercentOutput, 1);  // Pending to define speed...
  };
  void initializeLeftClimberMotor() {
    leftClimberMotor.Set(VictorSPXControlMode::PercentOutput, 1);  // Pending to define speed...
  };

  void desinitializeRightClimberMotor() {
    rightClimberMotor.Set(VictorSPXControlMode::PercentOutput, 0);  // Pending to define speed...
  };
  void desinitializeLeftClimberMotor() {
    leftClimberMotor.Set(VictorSPXControlMode::PercentOutput, 0);  // Pending to define speed...
  };
  void reverseRightClimberMotor() {
    rightClimberMotor.Set(VictorSPXControlMode::PercentOutput, -1);  // Pending to define speed...
  };
  void reverseLeftClimberMotor() {
    leftClimberMotor.Set(VictorSPXControlMode::PercentOutput, -1);  // Pending to define speed...
  };
  void climberSolenoidForward() {
    climberSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  };
  void climberSolenoidReverse() {
    climberSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  };
  
  //Limit Switch

  bool getSwitch(){
    if (climberSwitch.Get()){
      return true;
    } else {
      return false;
    }
  }

  void climberLimitswitch() {
    if (speed < 0 and getSwitch()){
      rightClimberMotor.SetVoltage(0_V);
      leftClimberMotor.SetVoltage(0_V);

   } else{
     rightClimberMotor.SetVoltage(units::volt_t(speed));
     leftClimberMotor.SetVoltage(units::volt_t(speed));

   }
  }
  void Periodic() override{};

 private:
  WPI_VictorSPX rightClimberMotor{10};
  WPI_VictorSPX leftClimberMotor{6};
  frc::DigitalInput climberSwitch {3};

  frc::DoubleSolenoid climberSolenoid{frc::PneumaticsModuleType::CTREPCM, 3, 2};
  double speed = 0;
};