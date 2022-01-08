// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "TurnToAngle.h"

TurnToAngle::TurnToAngle(Chassis* chassis, double angleObjective) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->chassis = chassis;
  this->angleObjective = angleObjective;
  AddRequirements(chassis);
}

// Called when the command is initially scheduled.
void TurnToAngle::Initialize() {

  turnToAnglePID.EnableContinuousInput(-180, 180);
  //frc::SmartDashboard::PutNumber("P", 0);
  //frc::SmartDashboard::PutNumber("I", 0);
  //frc::SmartDashboard::PutNumber("D", 0);
  //frc::SmartDashboard::PutNumber("targetAngle", 0);
}

// Called repeatedly when this Command is scheduled to run
void TurnToAngle::Execute() {
  turnToAnglePID.SetSetpoint(angleObjective);
  currentAngle = chassis->getPose().Rotation().Degrees().value();

  double angularVelocity = turnToAnglePID.Calculate(currentAngle);

  frc::ChassisSpeeds vels;
  vels.omega = units::radians_per_second_t(angularVelocity);
  chassis->setVelocities(vels);

  //double P = frc::SmartDashboard::GetNumber("P", 0);
  //double I = frc::SmartDashboard::GetNumber("I", 0);
  //double D = frc::SmartDashboard::GetNumber("D", 0); 
  //angleObjective = frc::SmartDashboard::GetNumber("targetAngle", 0);
  //turnToAnglePID.SetPID(P, I, D);

}

// Called once the command ends or is interrupted.
void TurnToAngle::End(bool interrupted) {
  frc::ChassisSpeeds vels;
  vels.omega = units::radians_per_second_t(0);
  chassis->setVelocities(vels);
}

// Returns true when the command should end.
bool TurnToAngle::IsFinished() {
  if (abs(currentAngle - angleObjective) < tolerance)
  {
    return true;
  } else{
    return false;
  }
}

