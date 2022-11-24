// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Subsystems/SwerveModule/SwerveModule.h>
#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <AHRS.h>

class SwerveChassis : public frc2::SubsystemBase
{
public:
  SwerveChassis()
  {
    backRightModule.setPIDvalues(0.09, 0.5, 0, 0);
    backLeftModule.setPIDvalues(0.09, 0.5, 0, 0);
    frontRightModule.setPIDvalues(0.09, 0.5, 0, 0);
    frontLeftModule.setPIDvalues(0.09, 0.5, 0, 0);

    navx.Calibrate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    double startTime = frc::Timer::GetFPGATimestamp().value();
    while (navx.IsCalibrating())
    {
      double timePassed = frc::Timer::GetFPGATimestamp().value() - startTime;
      if (timePassed > 10)
      {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    navx.ZeroYaw();
  }

  void setTargetAngle(double targetAngle)
  {
    this->targetAngle = targetAngle;
  }

  void setSpeed(double linearX, double linearY, double angular)
  {
    this->linearX = linearX;
    this->linearY = linearY;
    this->angular = angular;
  }

  void setWheelVoltage(double wheelVoltage)
  {
    this->wheelVoltage = wheelVoltage;
  }

  const frc::Pose2d &getOdometry()
  {
    return odometry.GetPose();
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override
  {

    frc::ChassisSpeeds chassisSpeed;

    frc::SmartDashboard::PutNumber("backLeftModule", backLeftModule.getAngle());
    frc::SmartDashboard::PutNumber("backRightModule", backRightModule.getAngle());
    frc::SmartDashboard::PutNumber("frontLeftModule", frontLeftModule.getAngle());
    frc::SmartDashboard::PutNumber("frontRightModule", frontRightModule.getAngle());

    chassisSpeed.vx = units::meters_per_second_t(linearX);
    chassisSpeed.vy = units::meters_per_second_t(linearY);
    chassisSpeed.omega = units::radians_per_second_t(angular);

    wpi::array<frc::SwerveModuleState, 4> desiredStates = kinematics.ToSwerveModuleStates(chassisSpeed);

    frontLeftModule.setAngle(desiredStates[0].angle.Degrees().value());
    frontRightModule.setAngle(desiredStates[1].angle.Degrees().value());
    backRightModule.setAngle(desiredStates[2].angle.Degrees().value());
    backLeftModule.setAngle(desiredStates[3].angle.Degrees().value());

    frontLeftModule.SetWheelVoltage(desiredStates[0].speed.value());
    frontRightModule.SetWheelVoltage(desiredStates[1].speed.value());
    backRightModule.SetWheelVoltage(desiredStates[2].speed.value());
    backLeftModule.SetWheelVoltage(desiredStates[3].speed.value());
    frc::SmartDashboard::PutNumber("frontLeftSpeed", desiredStates[0].speed.value());
    frc::SmartDashboard::PutNumber("frontRightSpeed", desiredStates[1].speed.value());
    frc::SmartDashboard::PutNumber("LinearX", linearX);
    frc::SmartDashboard::PutNumber("LinearY", linearY);
    frc::SmartDashboard::PutNumber("Angular", angular);

    backRightModule.Periodic();
    backLeftModule.Periodic();
    frontLeftModule.Periodic();
    frontRightModule.Periodic();

    odometry.Update(frc::Rotation2d(units::degree_t(-navx.GetAngle())), backRightModule.getState(), backLeftModule.getState(), frontLeftModule.getState(), frontRightModule.getState());

    frc::SmartDashboard::PutNumber("OdometryX", getOdometry().X().value());
    frc::SmartDashboard::PutNumber("OdometryY", getOdometry().Y().value());
    frc::SmartDashboard::PutNumber("AnglenaveX", -navx.GetAngle());
  }

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  SwerveModule backRightModule{1, 2, 9, -143.70507812500001};
  SwerveModule backLeftModule{3, 4, 10, -70};
  SwerveModule frontLeftModule{5, 6, 11, -147.5};
  SwerveModule frontRightModule{7, 8, 12, -160};

  double wheelVoltage;
  double targetAngle;

  double linearX;
  double linearY;
  double angular;

  std::array<frc::Translation2d, 4> modulePos{
      frc::Translation2d(10.36_in, 10.36_in),   // front left
      frc::Translation2d(10.36_in, -10.36_in),  // front right
      frc::Translation2d(-10.36_in, -10.36_in), // back right
      frc::Translation2d(-10.36_in, 10.36_in)   // back left
  };

  AHRS navx{frc::SPI::Port::kMXP};
  frc::SwerveDriveKinematics<4> kinematics{modulePos};

  frc::SwerveDriveOdometry<4> odometry{kinematics, frc::Rotation2d(0_deg)};
};
