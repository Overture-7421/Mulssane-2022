// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/controller/LinearQuadraticRegulator.h>


using namespace ctre::phoenix::motorcontrol::can;
class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();
  void setVelocity(double radsPerS);
  bool reachedVelocityTarget();
  double getVelocity();
  void Periodic() override;

 private:
  WPI_TalonFX rightShooter{5};
  WPI_TalonFX leftShooter{6};

  int encoder_CodesPerRev = 2048;


  double radsPerSecond = 0.0;
  double tolerance = 8;
  const double timeToStableRPS = 0.2;  // Seconds
  double lastTimeStable = 0;
  bool lastOnTargetState = false;

  
  // TODO Caracterizar disparador
  
  // Volts per (radian per second)
  static constexpr auto kFlywheelKv = 0.0_V / 1_rad_per_s; 

  // Volts per (radian per second squared)
  static constexpr auto kFlywheelKa = 0.0_V / 1_rad_per_s_sq;

  frc::LinearSystem<1, 1, 1> m_flywheelPlant =
      frc::LinearSystemId::IdentifyVelocitySystem<units::radian>(kFlywheelKv,
                                                                 kFlywheelKa);
  frc::KalmanFilter<1, 1, 1> m_observer{
      m_flywheelPlant,
      {3.0},   // How accurate we think our model is
      {0.01},  // How accurate we think our encoder data is
      20_ms};

  // A LQR uses feedback to create voltage commands.
  frc::LinearQuadraticRegulator<1, 1> m_controller{
      m_flywheelPlant,
      // qelms. Velocity error tolerance, in radians per second. Decrease this
      // to more heavily penalize state excursion, or make the controller behave
      // more aggressively.
      {tolerance},
      // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less
      // aggressive. 12 is a good starting point because that is the
      // (approximate) maximum voltage of a battery.
      {12.0},
      // Nominal time between loops. 20ms for TimedRobot, but can be lower if
      // using notifiers.
      20_ms};

  // The state-space loop combines a controller, observer, feedforward and plant
  // for easy control.
  frc::LinearSystemLoop<1, 1, 1> m_loop{m_flywheelPlant, m_controller,
                                        m_observer, 12_V, 20_ms};
};
