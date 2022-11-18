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

class SwerveChassis : public frc2::SubsystemBase
{
public:
  SwerveChassis();

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

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override
  {

    frc::ChassisSpeeds chassisSpeed;

    frc::SmartDashboard::PutNumber("backLeftModule", backLeftModule.returnPosition());
    frc::SmartDashboard::PutNumber("backRightModule", backRightModule.returnPosition());
    frc::SmartDashboard::PutNumber("frontLeftModule", frontLeftModule.returnPosition());
    frc::SmartDashboard::PutNumber("frontRightModule", frontRightModule.returnPosition());

    chassisSpeed.vx = units::meters_per_second_t(linearX);
    chassisSpeed.vy = units::meters_per_second_t(linearY);
    chassisSpeed.omega = units::radians_per_second_t(angular);

    frc::SwerveDriveKinematics wheelSpeeds = kinematics.ToSwerveModuleStates(chassisSpeed);

    backRightModule.setPIDvalues(0.125, 0.5, 0, 0);
    backLeftModule.setPIDvalues(0.125, 0.5, 0, 0);
    frontRightModule.setPIDvalues(0.125, 0.5, 0, 0);
    frontLeftModule.setPIDvalues(0.125, 0.5, 0, 0);

    backRightModule.setAngle(targetAngle);
    backLeftModule.setAngle(targetAngle);
    frontRightModule.setAngle(targetAngle);
    frontLeftModule.setAngle(targetAngle);

    backRightModule.SetWheelVoltage(wheelVoltage);
    backLeftModule.SetWheelVoltage(wheelVoltage);
    frontRightModule.SetWheelVoltage(wheelVoltage);
    frontLeftModule.SetWheelVoltage(wheelVoltage);

    backRightModule.Periodic();
    backLeftModule.Periodic();
    frontLeftModule.Periodic();
    frontRightModule.Periodic();
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
    frc::Translation2d(10.36_in, 10.36_in), //front left
    frc::Translation2d(10.36_in, -10.36_m), //front right
    frc::Translation2d(-10.36_m, -10.36_m), //back right
    frc::Translation2d(-10.36_m, 10.36_m)  //back left
  };

  frc::SwerveDriveKinematics<4> kinematics{modulePos};
};
