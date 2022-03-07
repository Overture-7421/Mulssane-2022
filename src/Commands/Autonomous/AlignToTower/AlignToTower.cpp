// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignToTower.h"

AlignToTower::AlignToTower(Chassis* chassis, VisionManager* visionManager) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->chassis = chassis;
  this->visionManager = visionManager;
  AddRequirements(chassis);
}

// Called when the command is initially scheduled.
void AlignToTower::Initialize() {
  turnToAnglePID.SetGoal(visionManager->getRotationToTarget().Degrees());
  turnToAnglePID.SetTolerance(4_deg);
  turnToAnglePID.EnableContinuousInput(-180_deg, 180_deg);
  turnToAnglePID.Reset(chassis->getPose().Rotation().Degrees());
}

// Called repeatedly when this Command is scheduled to run
void AlignToTower::Execute() {
  const auto currentAngle = chassis->getPose().Rotation().Degrees();
  turnToAnglePID.SetGoal(visionManager->getRotationToTarget().Degrees());
  double angularVelocity = turnToAnglePID.Calculate(currentAngle);

  frc::ChassisSpeeds vels;
  vels.omega = units::radians_per_second_t(angularVelocity);
  chassis->setVelocities(vels);
}

// Called once the command ends or is interrupted.
void AlignToTower::End(bool interrupted) {}

// Returns true when the command should end.
bool AlignToTower::IsFinished() { return turnToAnglePID.AtGoal(); }
