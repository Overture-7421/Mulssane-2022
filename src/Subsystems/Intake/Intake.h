// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/Phoenix.h>

class Intake : public frc2::SubsystemBase {
 public:
  Intake();
  void setPistons(bool set);
  void setVoltage(double voltage);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
   frc::DoubleSolenoid intakePiston{frc::PneumaticsModuleType::CTREPCM, 4, 5};
   WPI_TalonSRX intakeMotor{10}; // Intake

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
