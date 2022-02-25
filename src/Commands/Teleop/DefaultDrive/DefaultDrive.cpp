// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DefaultDrive.h"
#include "Utils/Utils.h"

DefaultDrive::DefaultDrive(Chassis* chassis, VisionManager* visionManager, frc::Joystick* joy){
    this->chassis = chassis;
    this->visionManager = visionManager;
    this->joy = joy;
    AddRequirements(chassis);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void DefaultDrive::Initialize() {
  headingController.EnableContinuousInput(units::degree_t(-180), units::degree_t(180));
  headingController.Reset(chassis->getPose().Rotation().Degrees());
  headingController.SetGoal(0_deg);

  frc::SmartDashboard::PutData("Heading PID", &headingController);
}

// Called repeatedly when this Command is scheduled to run
void DefaultDrive::Execute() {  
  frc::ChassisSpeeds vels;
  double linearAxis = Utils::ApplyAxisFilter(-joy->GetRawAxis(1));
  double angularAxisUnfiltered = Utils::ApplyAxisFilter(-joy->GetRawAxis(4), 0.1, 0.3);
  double angularAxis = angularAccelLimiter.Calculate(units::radians_per_second_t(angularAxisUnfiltered)).value();

  headingController.SetGoal(visionManager->getRotationToTarget().Degrees());
  double headingOut = headingController.Calculate(chassis->getPose().Rotation().Degrees());

  vels.vx = units::meters_per_second_t(linearAxis * chassis->getMaxVelocity());


  if(joy->GetRawButton(5)){
    vels.omega = units::radians_per_second_t(headingOut);
  }else{
    vels.omega = units::radians_per_second_t(angularAxis * 2 * M_PI); // Angular 
  }

  chassis->setVelocities(vels);
}

// Called once the command ends or is interrupted.
void DefaultDrive::End(bool interrupted) {}

// Returns true when the command should end.
// DEFAULT COMMANDS SHOULD ALWAYS RETURN FALSE
bool DefaultDrive::IsFinished() {
  return false;
}
