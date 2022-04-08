// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>

class Climber : public frc2::SubsystemBase {
 public:
  Climber();
  void setPistons(bool set);
  void setVoltage(double voltage);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
   frc::Solenoid 
   climberPiston{frc::PneumaticsModuleType::CTREPCM, 6};
   WPI_VictorSPX leftClimber {8}; // Left Climber INVERTED
   WPI_VictorSPX rightClimber {7}; // Right Climber
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
