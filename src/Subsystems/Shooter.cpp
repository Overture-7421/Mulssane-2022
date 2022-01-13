// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter() {
  rightShooter.SetInverted(InvertType::InvertMotorOutput);

  rightShooter.ConfigOpenloopRamp(0.01);
  leftShooter.ConfigOpenloopRamp(0.01);

  leftShooter.Follow(rightShooter);

  rightShooter.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
  leftShooter.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);

  rightShooter.SetSelectedSensorPosition(0.0);
  leftShooter.SetSelectedSensorPosition(0.0);
  frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
  frc::SmartDashboard::PutNumber("Shooter/Velocity", getVelocity());
  frc::SmartDashboard::PutNumber("Shooter/Position",
                                 rightShooter.GetSelectedSensorPosition());
  frc::SmartDashboard::PutBoolean("Shooter/ObjectiveReached",
                                  reachedVelocityTarget());
  frc::SmartDashboard::PutBoolean("Shooter/VoltageApplied",
                                  rightShooter.GetMotorOutputVoltage());

  m_loop.SetNextR(Eigen::Vector<double, 1>{radsPerSecond});
  m_loop.Correct(Eigen::Vector<double, 1>{getVelocity()});
  m_loop.Predict(20_ms);
  rightShooter.SetVoltage(units::volt_t(m_loop.U(0)));

  double currentTime = frc::Timer::GetFPGATimestamp().value();
  bool onTarget = abs(m_loop.Error()(0)) < tolerance;

  bool onTargetChanged = onTarget != lastOnTargetState;

  if (onTarget && onTargetChanged) {
    lastTimeStable = currentTime;
  }

  lastOnTargetState = onTarget;
  stabilizedOnTarget = currentTime - lastTimeStable > timeToStableRPS && onTarget;
}

void Shooter::setVelocity(double radsPerS) { this->radsPerSecond = radsPerS; }

bool Shooter::reachedVelocityTarget() {
  return stabilizedOnTarget;
}

double Shooter::getVelocity() {
  double encoderCodesPerSec = rightShooter.GetSelectedSensorVelocity() * 10;
  return (encoderCodesPerSec / encoder_CodesPerRev) * 2.0 * M_PI;
}
