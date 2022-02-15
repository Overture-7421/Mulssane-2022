// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "TurnToAngle.h"

TurnToAngle::TurnToAngle(Chassis* chassis, double angleObjective) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->chassis = chassis;
  this->angleObjective = angleObjective;

  
  turnToAnglePID.SetGoal(units::degree_t(angleObjective));
  turnToAnglePID.SetTolerance(2_deg, 900_deg_per_s);
  turnToAnglePID.EnableContinuousInput(-180_deg, 180_deg);
  frc::SmartDashboard::PutNumber("Plant Angle", 0.0);

  AddRequirements(chassis);
}

// Called when the command is initially scheduled.
void TurnToAngle::Initialize() {
  turnToAnglePID.Reset(chassis->getPose().Rotation().Degrees());
}

// Called repeatedly when this Command is scheduled to run
void TurnToAngle::Execute() {
  
  const auto currentAngle = chassis->getPose().Rotation().Degrees();
  frc::SmartDashboard::PutNumber("Plant Angle", currentAngle.value());
  double angularVelocity = turnToAnglePID.Calculate(currentAngle);

  frc::ChassisSpeeds vels;
  vels.omega = units::radians_per_second_t(angularVelocity);
  chassis->setVelocities(vels);
}

// Called once the command ends or is interrupted.
void TurnToAngle::End(bool interrupted) {
  frc::ChassisSpeeds vels;
  vels.omega = units::radians_per_second_t(0);
  chassis->setVelocities(vels);
}

// Returns true when the command should end.
bool TurnToAngle::IsFinished() {
  return turnToAnglePID.AtGoal();
}


