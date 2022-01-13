// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"
#define M_2_PI 6.2831852

Shooter::Shooter() {
    rightShooter.SetInverted(InvertType::InvertMotorOutput);

    rightShooter.ConfigOpenloopRamp(0.01);
    leftShooter.ConfigOpenloopRamp(0.01);

    rightShooter.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
    leftShooter.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor); 

    rightShooter.SetSelectedSensorPosition(0.0);
    leftShooter.SetSelectedSensorPosition(0.0);
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
    frc::SmartDashboard::PutNumber("Shooter/Velocity", getCurrentRPS());
    frc::SmartDashboard::PutNumber("Shooter/Position", rightShooter.GetSelectedSensorPosition());
    frc::SmartDashboard::PutBoolean("Shooter/ObjectiveReached", rpsObjectiveReached());

    double targetRPS = rpsRateLimiter.Calculate(units::radians_per_second_t(radsPerSecond)).to<double>();
    frc::SmartDashboard::PutNumber("Shooter/RateLimitedRPS", targetRPS);
    double pulsesPerSecond = encoder_CodesPerRev * targetRPS / M_2_PI;
    ShooterMaster.Set(ControlMode::Velocity, pulsesPerSecond / 10.0);
}

void Shooter::TestShoot(){

    leftShooter.Set(ControlMode::PercentOutput,0.3);
    rightShooter.Set(ControlMode::PercentOutput,0.3);
}

void Shooter::setRPS(double rps)
{
    this->radsPerSecond = rps;
}

bool Shooter::rpsObjectiveReached()
{
    double currentVelocity = getCurrentRPS();
    double error = radsPerSecond - currentVelocity;
    double currentTime = frc::Timer::GetFPGATimestamp();
    bool onTarget = abs(error) < tolerance;

    bool onTargetChanged = onTarget != lastOnTargetState;

    if (onTarget && onTargetChanged)
    {
        lastTimeStable = currentTime;
    }

    lastOnTargetState = onTarget;
    return currentTime - lastTimeStable > timeToStableRPS && onTarget;
}

double Shooter::getCurrentRPS()
{
    return (rightShooter.GetSelectedSensorVelocity() / encoder_CodesPerRev) * M_2_PI * 10;
}

