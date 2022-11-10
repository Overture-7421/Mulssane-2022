// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <iostream>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/filter/SlewRateLimiter.h>

class Chassis : public frc2::SubsystemBase {
 public:
  Chassis(){

    //Good slaves follow orders.
    leftSlave_1.Follow(chassisLeftMaster);
    leftSlave_2.Follow(chassisLeftMaster);
    rightSlave_1.Follow(chassisRightMaster);
    rightSlave_2.Follow(chassisRightMaster);

    //Right motors must be inverted.
    chassisRightMaster.SetInverted(true);
    rightSlave_1.SetInverted(true);
    rightSlave_2.SetInverted(true);

    //Creates FeedbackSensor configuration.
    chassisLeftMaster.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    chassisRightMaster.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);

    //Sensors must start at 0.
    chassisLeftMaster.SetSelectedSensorPosition(0);
    chassisRightMaster.SetSelectedSensorPosition(0);
  }

  //Receives doubles and sets it as the voltage.
  void wheelVoltage(double leftVoltage, double rightVoltage) {
    chassisLeftMaster.SetVoltage(units::volt_t(leftVoltage));
    chassisRightMaster.SetVoltage(units::volt_t(rightVoltage));
  }

  //saves received data into linear and angular speed values.
  void setSpeed(double linearSpeed, double angularSpeed) {
    this->linearSpeed = linearSpeed;
    this->angularSpeed = angularSpeed;
  }

  //Gives right speed in meters.
  double getLeftSpeed(){
    return getMeters(chassisLeftMaster.GetSelectedSensorVelocity() * 10);
  }

  //Gives right speed in meters.
  double getRightSpeed(){
    return getMeters(chassisRightMaster.GetSelectedSensorVelocity() * 10);
  }

  //Gives meter value of given data.
  double getMeters(double codes){
      double meters = codes / 2048 / 12 * 0.4787773;
      return meters;
  }

  //Returns distance travelled by left side.
  double getLeftDistance(){
    return getMeters(chassisLeftMaster.GetSelectedSensorPosition());
  }

  //Returns distance travelled by right side.
  double getRightDistance(){
    return getMeters(chassisRightMaster.GetSelectedSensorPosition());
  }

  //Returns PID value with given velocity and sets it to wheelVoltage().
  void getPID(double leftMPS, double rightMPS){
     double leftPID =  LeftPIDcontroller.Calculate(getLeftSpeed(), leftMPS) + feedforward.Calculate(units::meters_per_second_t(leftMPS)).value();
     double rightPID =  RightPIDcontroller.Calculate(getRightSpeed(), rightMPS) + feedforward.Calculate(units::meters_per_second_t(rightMPS)).value();
    wheelVoltage(leftPID, rightPID);
  }

  //Sets Proportional, Integral and Derivative values from given values.
  void setPIDvaluesLeft(double kP, double kI, double kD, double f){
    LeftPIDcontroller.SetPID(kP, kI, kD);
    leftF = f;
  }
  void setPIDvaluesRight(double kP, double kI, double kD, double f){
    RightPIDcontroller.SetPID(kP, kI, kD);
    rightF = f;
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override {

    frc::ChassisSpeeds chassisSpeed;

    //Determines the slew rate with given linear and angular speed.
    double srl = slewRateLinear.Calculate(units::meter_t(linearSpeed)).value();
    double sra = slewRateAngular.Calculate(units::radian_t(angularSpeed)).value();

    //Creates speeds with slewrates.
    chassisSpeed.vx = units::meters_per_second_t(srl);
    chassisSpeed.omega = units::radians_per_second_t(sra);
    frc::DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeed);

    //Returns PID.
    getPID(wheelSpeeds.left.value(), wheelSpeeds.right.value());
  }



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

  frc2::PIDController LeftPIDcontroller{0, 0, 0};
  frc2::PIDController RightPIDcontroller{0, 0, 0};

  double leftF = 2.9;
  double rightF = 2.75;

  double linearSpeed = 0;
  double angularSpeed = 0;

  frc::SimpleMotorFeedforward<units::meter> feedforward{0.59048_V, 2.5791_V / 1_mps, 0.62737_V / 1_mps_sq};

  frc::SlewRateLimiter<units::meter> slewRateLinear{4.5_m / 1_s};
  frc::SlewRateLimiter<units::radian> slewRateAngular{12_rad / 1_s};
};
