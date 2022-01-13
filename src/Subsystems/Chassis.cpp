/*
    ____                     __     ____        __          __     _   __                   
   /  _/___  ________  _____/ /_   / __ \____  / /_  ____  / /_   / | / /___ _____ ___  ___ 
   / // __ \/ ___/ _ \/ ___/ __/  / /_/ / __ \/ __ \/ __ \/ __/  /  |/ / __ `/ __ `__ \/ _ \
 _/ // / / (__  )  __/ /  / /_   / _, _/ /_/ / /_/ / /_/ / /_   / /|  / /_/ / / / / / /  __/
/___/_/ /_/____/\___/_/   \__/  /_/ |_|\____/_.___/\____/\__/  /_/ |_/\__,_/_/ /_/ /_/\___/                                               
*/

#include "Chassis.h"
#include <iostream>

Chassis::Chassis() {
  ahrs.Calibrate();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  double startTime = frc::Timer::GetFPGATimestamp().value();
  while (ahrs.IsCalibrating())
  {
    double timePassed = frc::Timer::GetFPGATimestamp().value() - startTime;
    if (timePassed > 10)
    {
      std::cout<< "ERROR!!!!: NavX took too long to calibrate." <<std::endl;
      break;
    }
     
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  ahrs.ZeroYaw();

  rightSlave1.Follow(rightMaster);

  leftMaster.SetInverted(InvertType::InvertMotorOutput);
  leftSlave1.SetInverted(InvertType::InvertMotorOutput);
  leftSlave1.Follow(leftMaster);

  rightMaster.ConfigOpenloopRamp(0.01);
  leftMaster.ConfigOpenloopRamp(0.01);

  rightMaster.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
  leftMaster.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor); 

  rightMaster.SetSelectedSensorPosition(0.0);
  leftMaster.SetSelectedSensorPosition(0.0);

   frc2::CommandScheduler::GetInstance().RegisterSubsystem(this); 
}

  const frc::Pose2d& Chassis::getPose(){
    return currentPose;
  }

  frc2::SequentialCommandGroup Chassis::getRamseteCommand(const std::vector<frc::Pose2d>& waypoints , frc::TrajectoryConfig config, bool reversed){

    config.SetReversed(reversed);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(kinematics);
    // Apply the constraint
    config.AddConstraint(kinematicsConstraints);

    auto targetTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(waypoints, config);
    auto ramseteController = frc::RamseteController();
    auto commandGroup = frc2::SequentialCommandGroup();

    commandGroup.AddCommands(
        frc2::RamseteCommand(
            targetTrajectory, [this]() { return getPose(); },
            ramseteController,
            kinematics,
            [this](units::meters_per_second_t leftVel, units::meters_per_second_t rightVel) { 
              frc::DifferentialDriveWheelSpeeds wheelSpeeds;
              wheelSpeeds.left = leftVel;
              wheelSpeeds.right = rightVel;
          
              setVelocities(kinematics.ToChassisSpeeds(wheelSpeeds)); 
            },
            {this}),
        frc2::InstantCommand(
            [this]() { setVelocities({}); }, {this}));
            
    return commandGroup;
  }

 void Chassis::setVelocities(frc::ChassisSpeeds vels){
    frc::DifferentialDriveWheelSpeeds wheelVels = kinematics.ToWheelSpeeds(vels);
    rightTargetVel = wheelVels.right.value();
    leftTargetVel = wheelVels.left.value();
 }

void Chassis::resetOdometry(frc::Pose2d pose){
  odometry.ResetPosition(pose, units::degree_t(-ahrs.GetYaw()));
  rightMaster.SetSelectedSensorPosition(0.0);
  leftMaster.SetSelectedSensorPosition(0.0);
}

// This method will be called once per scheduler run
void Chassis::Periodic() {
  updatePIDs();
  updateTelemetry();
  rightDistance = convertToMeters(rightMaster.GetSelectedSensorPosition());
  leftDistance = convertToMeters(leftMaster.GetSelectedSensorPosition());

  rightVel = convertToMetersPerSec(rightMaster.GetSelectedSensorVelocity());
  leftVel = convertToMetersPerSec(leftMaster.GetSelectedSensorVelocity());


  frc::Rotation2d gyroAngle{units::degree_t(-ahrs.GetYaw())};
  currentPose = odometry.Update(gyroAngle, units::meter_t(leftDistance), units::meter_t(rightDistance));
}

void Chassis::updatePIDs(){

  rightPID.SetSetpoint(rightTargetVel);
  leftPID.SetSetpoint(leftTargetVel);

  double leftOutput = leftPID.Calculate(leftVel);
  double rightOutput = rightPID.Calculate(rightVel);

  rightMaster.Set(ControlMode::PercentOutput, rightOutput + rightTargetVel * rightPIDF);
  leftMaster.Set(ControlMode::PercentOutput, leftOutput + leftTargetVel * leftPIDF);

}

void Chassis::updateTelemetry(){
  frc::SmartDashboard::PutNumber("Chassis/RightSensorRawPos", rightMaster.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Chassis/LeftSensorRawPos", leftMaster.GetSelectedSensorPosition());

  frc::SmartDashboard::PutNumber("Chassis/RightSensorMetersPos", rightDistance);
  frc::SmartDashboard::PutNumber("Chassis/LeftSensorMetersPos", leftDistance);

  frc::SmartDashboard::PutNumber("Chassis/RightVelocity", rightVel);
  frc::SmartDashboard::PutNumber("Chassis/LeftVelocity", leftVel);
  
  frc::SmartDashboard::PutNumber("Chassis/X", currentPose.X().to<double>());
  frc::SmartDashboard::PutNumber("Chassis/Y", currentPose.Y().to<double>());
  frc::SmartDashboard::PutNumber("Chassis/Yaw", currentPose.Rotation().Degrees().to<double>());
}
double Chassis::convertToMeters(double sensorRawPos){
  double sensorMeterPos = sensorRawPos / encoder_CodesPerRev * 2 * M_PI * wheelRadius;

  return sensorMeterPos;
}

double Chassis::convertToMetersPerSec(double rawEncoderVel100ms){
  double rawEncoderVel = rawEncoderVel100ms * 10;
  double revPerSec = rawEncoderVel / encoder_CodesPerRev;
  double angularVel = revPerSec * 2 * M_PI;
  double metersPerSec = angularVel * wheelRadius;

  return metersPerSec;
}
