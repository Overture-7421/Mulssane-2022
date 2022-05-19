
#include "Chassis.h"

#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

#include <iostream>

Chassis::Chassis() {
  ahrs.Calibrate();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  double startTime = frc::Timer::GetFPGATimestamp().value();
  while (ahrs.IsCalibrating()) {
    double timePassed = frc::Timer::GetFPGATimestamp().value() - startTime;
    if (timePassed > 10) {
      std::cout << "ERROR!!!!: NavX took too long to calibrate." << std::endl;
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ahrs.ZeroYaw();
  rightSlave1.Follow(rightMaster);
  rightSlave2.Follow(rightMaster);

  rightMaster.SetInverted(InvertType::InvertMotorOutput);
  rightSlave1.SetInverted(InvertType::InvertMotorOutput);
  rightSlave2.SetInverted(InvertType::InvertMotorOutput);

  leftSlave1.Follow(leftMaster);
  leftSlave2.Follow(leftMaster);

  rightMaster.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
  leftMaster.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);

  rightMaster.SetSelectedSensorPosition(0.0);
  leftMaster.SetSelectedSensorPosition(0.0);

  rightMaster.SetNeutralMode(NeutralMode::Brake);
  rightSlave1.SetNeutralMode(NeutralMode::Brake);
  rightSlave2.SetNeutralMode(NeutralMode::Brake);
  leftMaster.SetNeutralMode(NeutralMode::Brake);
  leftSlave1.SetNeutralMode(NeutralMode::Brake);
  leftSlave2.SetNeutralMode(NeutralMode::Brake);

  rightMaster.ConfigSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration(true, 30, 0, 1));
  rightSlave1.ConfigSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration(true, 30, 0, 1));
  rightSlave2.ConfigSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration(true, 30, 0, 1));

  leftMaster.ConfigSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration(true, 30, 0, 1));
  leftSlave1.ConfigSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration(true, 30, 0, 1));
  leftSlave2.ConfigSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration(true, 30, 0, 1));

  leftSlave1.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 255);
  leftSlave1.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
      255);

  leftSlave2.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 255);
  leftSlave2.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
      255);

  rightSlave1.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 255);
  rightSlave1.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
      255);

  rightSlave2.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 255);
  rightSlave2.SetStatusFramePeriod(
      ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
      255);

  frc::SmartDashboard::PutData("Chassis/RobotPose", &field);
  frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

frc::Pose2d Chassis::getPose() {
  pe_NMutex.lock();
  std::lock_guard<std::mutex> lg(pe_MMutex);
  pe_NMutex.unlock();
  return odometry.GetEstimatedPosition();
}

frc2::SequentialCommandGroup Chassis::getRamseteCommand(
    const std::vector<frc::Pose2d>& waypoints, frc::TrajectoryConfig config,
    bool reversed) {
  config.SetReversed(reversed);

  // config.AddConstraint(frc::CentripetalAccelerationConstraint(
  //     units::meters_per_second_squared_t(5)));

  auto targetTrajectory =
      frc::TrajectoryGenerator::GenerateTrajectory(waypoints, config);
  auto ramseteController = frc::RamseteController();
  auto commandGroup = frc2::SequentialCommandGroup();

  commandGroup.AddCommands(
      frc2::RamseteCommand(
          targetTrajectory, [this]() { return getPose(); }, ramseteController,
          kinematics,
          [this](units::meters_per_second_t leftVel,
                 units::meters_per_second_t rightVel) {
            frc::DifferentialDriveWheelSpeeds wheelSpeeds;
            wheelSpeeds.left = leftVel;
            wheelSpeeds.right = rightVel;

            setVelocities(kinematics.ToChassisSpeeds(wheelSpeeds));
          },
          {this}),
      frc2::InstantCommand([this]() { setVelocities({}); }, {this}));

  return commandGroup;
}

void Chassis::setVelocities(frc::ChassisSpeeds vels) {
  frc::DifferentialDriveWheelSpeeds wheelVels = kinematics.ToWheelSpeeds(vels);

  wheelVels.Desaturate(units::meters_per_second_t(getMaxVelocity()));

  leftTargetVel = wheelVels.left.value();
  rightTargetVel = wheelVels.right.value();
}

void Chassis::resetOdometry(frc::Pose2d pose) {
  setYaw(pose.Rotation().Degrees().value());
  pe_NMutex.lock();
  pe_MMutex.lock();
  pe_NMutex.unlock();

  odometry.ResetPosition(pose, pose.Rotation());
  leftMaster.SetSelectedSensorPosition(0.0);
  rightMaster.SetSelectedSensorPosition(0.0);
  pe_MMutex.unlock();
}

double Chassis::getMaxVelocity() { return maxSpeed; }

void Chassis::addVisionMeasurement(const frc::Pose2d& visionPose,
                                   double timeStamp) {
  // Low priority lock
  pe_LMutex.lock();
  pe_NMutex.lock();
  pe_MMutex.lock();
  pe_NMutex.unlock();
  odometry.AddVisionMeasurement(visionPose, units::second_t(timeStamp));
  pe_MMutex.unlock();
  pe_LMutex.unlock();
}

// This method will be called once per scheduler run
void Chassis::Periodic() {
  const auto start = frc::Timer::GetFPGATimestamp();

  updatePIDs();
  updateTelemetry();
  rightDistance = convertToMeters(rightMaster.GetSelectedSensorPosition());
  leftDistance = convertToMeters(leftMaster.GetSelectedSensorPosition());

  rightVel = convertToMetersPerSec(rightMaster.GetSelectedSensorVelocity());
  leftVel = convertToMetersPerSec(leftMaster.GetSelectedSensorVelocity());

  frc::Rotation2d gyroAngle{units::degree_t(-ahrs.GetYaw() - headingOffset)};
  frc::SmartDashboard::PutNumber("Chassis/RawHeading",
                                 gyroAngle.Degrees().value());

  frc::DifferentialDriveWheelSpeeds wheelSpeeds;
  wheelSpeeds.left = units::meters_per_second_t(leftVel);
  wheelSpeeds.right = units::meters_per_second_t(rightVel);
  pe_NMutex.lock();
  pe_MMutex.lock();
  pe_NMutex.unlock();
  odometry.Update(gyroAngle, wheelSpeeds, units::meter_t(leftDistance),
                  units::meter_t(rightDistance));
  pe_MMutex.unlock();

  frc::SmartDashboard::PutNumber(
      "Chassis/dt", (frc::Timer::GetFPGATimestamp() - start).value());
}

void Chassis::setYaw(double deegres) {
  headingOffset = -ahrs.GetYaw() - deegres;
}

void Chassis::updatePIDs() {
  const auto leftVelTargetMps = units::meters_per_second_t(leftTargetVel);
  const auto rightVelTargetMps = units::meters_per_second_t(rightTargetVel);

  const auto leftLimitedTarget = leftAccelLimiter.Calculate(leftVelTargetMps);
  const auto rightLimitedTarget =
      rightAccelLimiter.Calculate(rightVelTargetMps);

  leftPID.SetSetpoint(leftLimitedTarget.value());
  rightPID.SetSetpoint(rightLimitedTarget.value());

  const auto leftOutput = units::volt_t(leftPID.Calculate(leftVel)) +
                          ff.Calculate(leftLimitedTarget);
  const auto rightOutput = units::volt_t(rightPID.Calculate(rightVel)) +
                           ff.Calculate(rightLimitedTarget);

  leftMaster.SetVoltage(leftOutput);
  rightMaster.SetVoltage(rightOutput);

  frc::SmartDashboard::PutNumber("Chassis/RightOut", rightOutput.value());
  frc::SmartDashboard::PutNumber("Chassis/LeftOut", leftOutput.value());
}

void Chassis::updateTelemetry() {
  frc::SmartDashboard::PutNumber("Chassis/RightSensorRawPos",
                                 rightMaster.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Chassis/LeftSensorRawPos",
                                 leftMaster.GetSelectedSensorPosition());

  frc::SmartDashboard::PutNumber("Chassis/RightSensorMetersPos", rightDistance);
  frc::SmartDashboard::PutNumber("Chassis/LeftSensorMetersPos", leftDistance);

  frc::SmartDashboard::PutNumber("Chassis/RightVelocity", rightVel);
  frc::SmartDashboard::PutNumber("Chassis/LeftVelocity", leftVel);
  field.SetRobotPose(getPose());
}

double Chassis::convertToMeters(double sensorRawPos) {
  double sensorMeterPos =
      sensorRawPos / encoder_CodesPerRev * 2 * M_PI * wheelRadius;

  return sensorMeterPos;
}

double Chassis::convertToMetersPerSec(double rawEncoderVel100ms) {
  double rawEncoderVel = rawEncoderVel100ms * 10;
  double revPerSec = rawEncoderVel / encoder_CodesPerRev;
  double angularVel = revPerSec * 2 * M_PI;
  double metersPerSec = angularVel * wheelRadius;

  return metersPerSec;
}
