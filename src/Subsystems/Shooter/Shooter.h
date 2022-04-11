// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include <frc/DoubleSolenoid.h>

using namespace ctre::phoenix::motorcontrol::can;
class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();
  void setVelocity(double radsPerS);
  void setHoodState(bool set);
  bool reachedVelocityTarget();
  double getVelocity();
  void Periodic() override;

 private:
  WPI_TalonFX rightShooter{10};
  WPI_TalonFX leftShooter{4};

  int encoder_CodesPerRev = 2048;

  double radsPerSecond = 0.0;
  double tolerance = 15;

  // TODO Caracterizar disparador
  frc::SlewRateLimiter<units::radian> limiter{400_rad_per_s};

  // Volts
  static constexpr auto kFlywheelKs = 0.68103_V;

  // Volts per (radian per second)
  static constexpr auto kFlywheelKv = 0.018611_V / 1_rad_per_s;

  // Volts per (radian per second squared)
  static constexpr auto kFlywheelKa = 0.0021683_V / 1_rad_per_s_sq;

  frc2::PIDController shooterController{0.003936, 0, 0};
  frc::SimpleMotorFeedforward<units::radian> shooterFF{kFlywheelKs, kFlywheelKv,
                                                       kFlywheelKa};
  frc::DoubleSolenoid hoodPiston{frc::PneumaticsModuleType::CTREPCM, 4, 5};
};
