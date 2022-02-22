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
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
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

  void addVisionMeasurement(const frc::Pose2d& visionPose, double timeStamp);
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
   void updatePIDs();
   void updateTelemetry();
   double convertToMeters(double sensorRawPosition);
  double convertToMetersPerSec(double rawEncoderVel);


  WPI_TalonFX rightMaster {16}; 
  WPI_TalonFX rightSlave1 {15};
  WPI_TalonFX rightSlave2 {16};

  WPI_TalonFX leftMaster {3}; 
  WPI_TalonFX leftSlave1 {2};
  WPI_TalonFX leftSlave2 {3};

  const double wheelRadius = 0.0762; //metros
  const int encoder_CodesPerRev = 2048 * 12; //2048 Flacon * 12 por la reduccion

  double rightTargetVel = 0.0;
  double leftTargetVel = 0.0;

  double rightVel = 0;
  double leftVel = 0;

  double rightDistance = 0;
  double leftDistance = 0;

  AHRS ahrs{frc::SPI::Port::kMXP};

  mutable std::mutex pe_MMutex, pe_NMutex, pe_LMutex; //https://stackoverflow.com/questions/11666610/how-to-give-priority-to-privileged-thread-in-mutex-locking

  frc::DifferentialDriveKinematics kinematics {0.72_m};

  frc::DifferentialDrivePoseEstimator odometry {
    0_deg, frc::Pose2d{0_m, 0_m, 0_rad},
      wpi::array<double, 5>{0.035, 0.035, 0.035, 0.035, 0.035}, 
      wpi::array<double, 3>{0.001, 0.001, 0.001},
      wpi::array<double, 3>{0.01, 0.01, 0.01}
  };

  const double maxSpeed = 3.5; // Meters per second
  const double maxAcceleration = 30.0; // Meters per second squared
  
  frc::SlewRateLimiter<units::meters_per_second> rightAccelLimiter {units::meters_per_second_squared_t(maxAcceleration)};
  frc::SlewRateLimiter<units::meters_per_second> leftAccelLimiter {units::meters_per_second_squared_t(maxAcceleration)};

  frc2::PIDController rightPID {0.13065, 0, 0};
  frc2::PIDController leftPID {0.13065, 0, 0}; 
//0.13065
  //New and best Characterization.
  frc::SimpleMotorFeedforward<units::meter> ff {0.57851_V, 2.633_V / 1_mps, 0.13305_V / 1_mps_sq};

  frc::Pose2d currentPose{0_m, 0_m, 0_rad};
  
  frc::RamseteController ramsete;

};
