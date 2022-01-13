// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();
  void TestShoot();
  void setRPS(double rps);
  bool rpsObjectiveReached();
  double getCurrentRPS();
  void Periodic() override;

 private:
  
  TalonFX rightShooter {5};
  TalonFX leftShooter {6};

  int encoder_CodesPerRev = 10240;
  frc::SlewRateLimiter<units::radians_per_second> rpsRateLimiter{units::radians_per_second_t(50) / units::second_t(1)};
  double radsPerSecond = 0.0;
  double targetWidth = 0;
  double tolerance = 5;
  const double timeToStableRPS = 0.2; //Seconds
  double lastTimeStable = frc::Timer::GetFPGATimestamp();
  bool lastOnTargetState = false;

};
