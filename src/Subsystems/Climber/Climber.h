// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>

class Climber : public frc2::SubsystemBase {
 public:
  Climber();
  void invertClimberMotors() {
    rightClimberMotor.SetInverted(true);
    leftClimberMotor.SetInverted(true);
  };

  void initializeRightClimberMotor() {
    rightClimberMotor.Set(VictorSPXControlMode::PercentOutput,
                          1);  // Pending to define speed...
  };
  void initializeLeftClimberMotor() {
    leftClimberMotor.Set(VictorSPXControlMode::PercentOutput,
                         1);  // Pending to define speed...
  };

  void desinitializeRightClimberMotor() {
    rightClimberMotor.Set(VictorSPXControlMode::PercentOutput,
                          0);  // Pending to define speed...
  };
  void desinitializeLeftClimberMotor() {
    leftClimberMotor.Set(VictorSPXControlMode::PercentOutput,
                         0);  // Pending to define speed...
  };
  void reverseRightClimberMotor() {
    rightClimberMotor.Set(VictorSPXControlMode::PercentOutput,
                          -1);  // Pending to define speed...
  };
  void reverseLeftClimberMotor() {
    leftClimberMotor.Set(VictorSPXControlMode::PercentOutput,
                         -1);  // Pending to define speed...
  };

  void climberSolenoidForward() {
    climberSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  };
  void climberSolenoidReverse() {
    climberSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  };

  void Periodic() override;

 private:
  WPI_VictorSPX rightClimberMotor{10};
  WPI_VictorSPX leftClimberMotor{6};

  frc::DoubleSolenoid climberSolenoid{frc::PneumaticsModuleType::CTREPCM, 3, 2};
};