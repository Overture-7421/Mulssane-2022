// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <iostream>

class Chassis : public frc2::SubsystemBase {
 public:
  Chassis(){
    leftSlave_1.Follow(chassisLeftMaster);
    leftSlave_2.Follow(chassisLeftMaster);

    rightSlave_1.Follow(chassisRightMaster);
    rightSlave_2.Follow(chassisRightMaster);

    chassisRightMaster.SetInverted(true);
    rightSlave_1.SetInverted(true);
    rightSlave_2.SetInverted(true);

    chassisLeftMaster.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    chassisRightMaster.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);

    chassisLeftMaster.SetSelectedSensorPosition(0);
    chassisRightMaster.SetSelectedSensorPosition(0);
  }

  void wheelVoltage(double leftVoltage, double rightVoltage) {
    chassisLeftMaster.SetVoltage(units::volt_t(leftVoltage));
    chassisRightMaster.SetVoltage(units::volt_t(rightVoltage));
  }

  void setSpeed(double linearSpeed, double angularSpeed) {
    frc::ChassisSpeeds chassisSpeed;
    chassisSpeed.vx = units::meters_per_second_t(linearSpeed);
    chassisSpeed.omega = units::radians_per_second_t(angularSpeed);

    frc::DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeed);

    wheelVoltage(wheelSpeeds.left.value(), wheelSpeeds.right.value());
  }

  

  void getEncoder(){
    std::string leftData = std::to_string(getDistance(chassisLeftMaster.GetSelectedSensorPosition()));
    std::string rightData = std::to_string(getDistance(chassisRightMaster.GetSelectedSensorPosition()));

    std::cout << "Left: " << leftData << " Right: " << rightData << std::endl;
  }

  double getDistance(double position){
      double distance = position / 2048 / 12 * 0.4787773;
      return distance;
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;



 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX chassisLeftMaster{1};
  WPI_TalonFX leftSlave_1{2}; 
  WPI_TalonFX leftSlave_2{3};

  WPI_TalonFX chassisRightMaster{11};
  WPI_TalonFX rightSlave_1{12}; 
  WPI_TalonFX rightSlave_2{13}; 
   
  frc::DifferentialDriveKinematics kinematics{0.69_m};
};
