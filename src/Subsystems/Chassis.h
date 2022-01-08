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

class Chassis : public frc2::SubsystemBase {
 public:
  Chassis();

 const frc::Pose2d& getPose();
  frc2::SequentialCommandGroup getRamseteCommand(const std::vector<frc::Pose2d>& waypoints , frc::TrajectoryConfig config, bool reversed = false);

  void setVelocities(frc::ChassisSpeeds vels); // Speed has no direction D:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
   void updatePIDs();
   void updateTelemetry();
   double convertToMeters(double sensorRawPosition);
  double convertToMetersPerSec(double rawEncoderVel);


  TalonFX rightMaster {1};
  TalonFX rightSlave1 {3};

  TalonFX leftMaster {2};
  TalonFX leftSlave1 {4};

  double wheelRadius = 0.0508; //metros
  int encoder_CodesPerRev = 10240;

  frc2::PIDController rightPID {0.15, 0, 0};
  frc2::PIDController leftPID {0.15, 0, 0};

  double rightPIDF = 0.13;
  double leftPIDF = 0.13;

  double rightTargetVel = 0.0;
  double leftTargetVel = 0.0;

  double rightVel = 0;
  double leftVel = 0;

  double rightDistance = 0;
  double leftDistance = 0;

  AHRS ahrs{frc::SPI::Port::kMXP};

  frc::DifferentialDriveOdometry odometry{0_deg, frc::Pose2d{0_m, 0_m, 0_rad}};
  frc::DifferentialDriveKinematics kinematics {0.77_m}; // Tamaño correcto
  frc::DifferentialDriveKinematicsConstraint kinematicsConstraints {kinematics, 3_mps};

  frc::Pose2d currentPose{0_m, 0_m, 0_rad};
  
  frc::RamseteController ramsete;

};
