/*
    __  _____  ____   __________ ___    _   ________   ____        __          __     ______          __   
   /  |/  / / / / /  / ___/ ___//   |  / | / / ____/  / __ \____  / /_  ____  / /_   / ____/___  ____/ /__ 
  / /|_/ / / / / /   \__ \\__ \/ /| | /  |/ / __/    / /_/ / __ \/ __ \/ __ \/ __/  / /   / __ \/ __  / _ \
 / /  / / /_/ / /______/ /__/ / ___ |/ /|  / /___   / _, _/ /_/ / /_/ / /_/ / /_   / /___/ /_/ / /_/ /  __/
/_/  /_/\____/_____/____/____/_/  |_/_/ |_/_____/  /_/ |_|\____/_.___/\____/\__/   \____/\____/\__,_/\___/ 
                                                                                                           
*/

#include "DefaultDrive.h"

#include "Utils/Utils.h"

DefaultDrive::DefaultDrive(Chassis* chassis, VisionManager* visionManager,
                           RangeDecider* rangeDecider, frc::Joystick* joy) {
  this->chassis = chassis;
  this->visionManager = visionManager;
  this->rangeDecider = rangeDecider;
  this->joy = joy;
  AddRequirements(chassis);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void DefaultDrive::Initialize() {
  headingController.EnableContinuousInput(units::degree_t(-180),
                                          units::degree_t(180));
  headingController.Reset(chassis->getPose().Rotation().Degrees());
  headingController.SetTolerance(3_deg);

  frc::SmartDashboard::PutData("Heading PID", &headingController);
}

// Called repeatedly when this Command is scheduled to run
void DefaultDrive::Execute() {
  frc::ChassisSpeeds vels;
  double linearAxis = Utils::ApplyAxisFilter(-joy->GetRawAxis(1), 0.2);
  double angularAxis = Utils::ApplyAxisFilter(-joy->GetRawAxis(4), 0.125, 1);

  headingController.SetGoal(visionManager->getRotationToTarget().Degrees());
  double headingOut =
      headingController.Calculate(chassis->getPose().Rotation().Degrees());
  vels.vx = linearLimiter.Calculate(
      units::meters_per_second_t(linearAxis * chassis->getMaxVelocity()));

  if (joy->GetRawButton(aimButton)) {
    vels.omega = units::radians_per_second_t(headingOut);
  } else {
    vels.omega =
        angularLimiter.Calculate(units::radians_per_second_t(angularAxis * 2.5 * M_PI));  // Angular
  }

  chassis->setVelocities(vels);
}

// Called once the command ends or is interrupted.
void DefaultDrive::End(bool interrupted) {}

// Returns true when the command should end.
// DEFAULT COMMANDS SHOULD ALWAYS RETURN FALSE
bool DefaultDrive::IsFinished() { return false; }
