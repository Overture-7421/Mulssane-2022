// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/TimedRobot.h>

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  void Shoot() {shooter_Left.Set(-.5);
                        shooter_Right.Set(.5); }
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  WPI_TalonFX shooter_Left{14};
  WPI_TalonFX shooter_Right{15};


  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
