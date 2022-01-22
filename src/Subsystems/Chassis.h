/*
    ____                     __     ____        __          __     _   __                   
   /  _/___  ________  _____/ /_   / __ \____  / /_  ____  / /_   / | / /___ _____ ___  ___ 
   / // __ \/ ___/ _ \/ ___/ __/  / /_/ / __ \/ __ \/ __ \/ __/  /  |/ / __ `/ __ `__ \/ _ \
 _/ // / / (__  )  __/ /  / /_   / _, _/ /_/ / /_/ / /_/ / /_   / /|  / /_/ / / / / / /  __/
/___/_/ /_/____/\___/_/   \__/  /_/ |_|\____/_.___/\____/\__/  /_/ |_/\__,_/_/ /_/ /_/\___/                                               
*/

#pragma once

#define M_PI 3.14159265358979323846

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <frc/controller/PIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <AHRS.h>
#include <frc/controller/RamseteController.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/controller/RamseteController.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/SimpleMotorFeedforward.h>

class Chassis : public frc2::SubsystemBase {
 public:
  Chassis();

 const frc::Pose2d& getPose();
  frc2::SequentialCommandGroup getRamseteCommand(const std::vector<frc::Pose2d>& waypoints , frc::TrajectoryConfig config, bool reversed = false);

  void setVelocities(frc::ChassisSpeeds vels); // Speed has no direction D:

  void resetOdometry(frc::Pose2d pose = {});

  double getMaxVelocity();
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
   void updatePIDs();
   void updateTelemetry();
   double convertToMeters(double sensorRawPosition);
  double convertToMetersPerSec(double rawEncoderVel);


  WPI_TalonFX rightMaster {1};
  WPI_TalonFX rightSlave1 {2};
  WPI_TalonFX rightSlave2 {3};

  WPI_TalonFX leftMaster {6};
  WPI_TalonFX leftSlave1 {7};
  WPI_TalonFX leftSlave2 {8};

  const double wheelRadius = 0.0762; //metros
  const int encoder_CodesPerRev = 2048 * 12; //2048 Flacon * 12 por la reduccion

  double rightTargetVel = 0.0;
  double leftTargetVel = 0.0;

  double rightVel = 0;
  double leftVel = 0;

  double rightDistance = 0;
  double leftDistance = 0;

  AHRS ahrs{frc::SPI::Port::kMXP};

  frc::DifferentialDriveOdometry odometry{0_deg, frc::Pose2d{0_m, 0_m, 0_rad}};
  frc::DifferentialDriveKinematics kinematics {0.77_m}; // Tamaño correcto

  const double maxSpeed = 4.0; // Meters per second
  const double maxAcceleration = 40.0; // Meters per second squared
  
  frc::DifferentialDriveKinematicsConstraint kinematicsConstraints {kinematics, units::meters_per_second_t(maxSpeed)};
  frc::SlewRateLimiter<units::meters_per_second> rightAccelLimiter {units::meters_per_second_squared_t(maxAcceleration)};
  frc::SlewRateLimiter<units::meters_per_second> leftAccelLimiter {units::meters_per_second_squared_t(maxAcceleration)};

  frc2::PIDController rightPID {0, 0, 0};
  frc2::PIDController leftPID {0, 0, 0};

  frc::SimpleMotorFeedforward<units::meter> ff {0_V, 0_V / 1_mps, 0_V / 1_mps_sq};


  frc::Pose2d currentPose{0_m, 0_m, 0_rad};
  
  frc::RamseteController ramsete;

};
