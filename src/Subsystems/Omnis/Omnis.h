// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

class Omnis : public frc2::SubsystemBase {
 public:
  Omnis();



  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
 
 
 WPI_VictorSPX omnisMotor {4};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};