// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Shooter.h"

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter() {
  leftShooter.SetInverted(InvertType::InvertMotorOutput);

  rightShooter.ConfigOpenloopRamp(0.01);
  leftShooter.ConfigOpenloopRamp(0.01);

  leftShooter.Follow(rightShooter);

  rightShooter.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
  leftShooter.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);

  rightShooter.SetSelectedSensorPosition(0.0);
  leftShooter.SetSelectedSensorPosition(0.0);

  leftShooter.SetNeutralMode(NeutralMode::Coast);
  rightShooter.SetNeutralMode(NeutralMode::Coast);

  leftShooter.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 255);
  leftShooter.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
      255);

  shooterController.SetTolerance(tolerance);

  leftShooter.ConfigSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration(true, 30, 0, 1));
  rightShooter.ConfigSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration(true, 30, 0, 1));

  frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
  double currentVel = getVelocity();
  frc::SmartDashboard::PutNumber("Shooter/Velocity", currentVel);
  
  frc::SmartDashboard::PutNumber("Shooter/Target", shooterController.GetSetpoint());

  frc::SmartDashboard::PutBoolean("Shooter/TargetReached",
                                  reachedVelocityTarget());
  const double limitedSetpoint =
      limiter.Calculate(units::radian_t(radsPerSecond)).value();

  shooterController.SetSetpoint(limitedSetpoint);
  const auto pidOut = units::volt_t(shooterController.Calculate(currentVel));
  const auto ff = shooterFF.Calculate(units::radians_per_second_t(limitedSetpoint));
    frc::SmartDashboard::PutNumber("Shooter/FFVoltage",ff.value());
  rightShooter.SetVoltage(
      pidOut + ff);
}

void Shooter::setVelocity(double radsPerS) { this->radsPerSecond = radsPerS; }

void Shooter::setHoodState(bool set) {
  if (set) {
    hoodPiston.Set(frc::DoubleSolenoid::kForward);
  } else {
    hoodPiston.Set(frc::DoubleSolenoid::kReverse);
  }
}

bool Shooter::reachedVelocityTarget() { return shooterController.AtSetpoint(); }

double Shooter::getVelocity() {
  double encoderCodesPerSec = rightShooter.GetSelectedSensorVelocity() * 10;
  return (encoderCodesPerSec / encoder_CodesPerRev) * 2.0 * M_PI;
}
