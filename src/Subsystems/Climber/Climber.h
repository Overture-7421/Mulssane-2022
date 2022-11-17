// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>

class Climber : public frc2::SubsystemBase {
 public:
  Climber() {
    rightClimberMotor.SetInverted(true);
    leftClimberMotor.SetInverted(true);
  }

  // climber solenoids
  void setPistons(bool state) {
    if (state) {
      climberSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
      climberSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
  }

  // climber motors
  void setMotor(double voltage) {
    rightClimberMotor.SetVoltage(units::volt_t(voltage));
    leftClimberMotor.SetVoltage(units::volt_t(voltage));
  }

  void Periodic() override{};

 private:
  WPI_VictorSPX rightClimberMotor{10};
  WPI_VictorSPX leftClimberMotor{6};
  frc::DigitalInput climberSwitch{3};

  frc::DoubleSolenoid climberSolenoid{frc::PneumaticsModuleType::CTREPCM, 3, 2};
  double speed = 0;
};