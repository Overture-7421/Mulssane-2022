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
  double currentVel = getVelocity();
  frc::SmartDashboard::PutNumber("Shooter/Velocity", currentVel);
  frc::SmartDashboard::PutNumber("Shooter/Position",
                                 rightShooter.GetSelectedSensorPosition());
  frc::SmartDashboard::PutBoolean("Shooter/ObjectiveReached",
                                  reachedVelocityTarget());

  const double limitedSetpoint =
      limiter.Calculate(units::radian_t(radsPerSecond)).value();
  shooterController.SetSetpoint(limitedSetpoint);
  const auto pidOut = units::volt_t(shooterController.Calculate(currentVel));
  rightShooter.SetVoltage(
      pidOut +
      shooterFF.Calculate(units::radians_per_second_t(limitedSetpoint)));

  double currentTime = frc::Timer::GetFPGATimestamp().value();
  bool onTarget = abs(radsPerSecond - currentVel) < tolerance;

  bool onTargetChanged = onTarget != lastOnTargetState;

  if (onTarget && onTargetChanged) {
    lastTimeStable = currentTime;
  }

  lastOnTargetState = onTarget;
  stabilizedOnTarget =
      currentTime - lastTimeStable > timeToStableRPS && onTarget;
}

void Shooter::setVelocity(double radsPerS) { this->radsPerSecond = radsPerS; }

void Shooter::setHoodState(bool set) {
  if(set){
    hoodPiston.Set(frc::DoubleSolenoid::kForward);
  } else {
    hoodPiston.Set(frc::DoubleSolenoid::kReverse);
  }
}

bool Shooter::reachedVelocityTarget() { return stabilizedOnTarget; }

double Shooter::getVelocity() {
  double encoderCodesPerSec = rightShooter.GetSelectedSensorVelocity() * 10;
  return (encoderCodesPerSec / encoder_CodesPerRev) * 2.0 * M_PI;
}
