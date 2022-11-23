// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>

class SwerveModule {
 public:
  SwerveModule(int rotatorID, int wheelID, int canCoderID, double offSet) : rotator(rotatorID), wheel(wheelID), canCoder(canCoderID){
    canCoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);

    rotator.ConfigFactoryDefault();
    wheel.ConfigFactoryDefault();
    rotatorPID.EnableContinuousInput(-180, 180);
    canCoder.ConfigSensorDirection(false);
    rotator.SetInverted(true);

    this->offSet = offSet;
  }
  
  double getSpeed(){
    return getMeters(wheel.GetSelectedSensorVelocity() * 10);
  }

  double getMeters(double codes){
      double meters = codes / 2048 / 6.75 * 0.319024;
      return meters;
  }

  void SetRotatorVoltage(double rotatorVoltage) {
    rotator.SetVoltage(units::volt_t(rotatorVoltage));
  }

  void SetWheelVoltage(double wheelVoltage){
    wheel.SetVoltage(units::volt_t(wheelVoltage));
  }

  double getAngle(){
    return frc::Rotation2d(units::degree_t(canCoder.GetAbsolutePosition())).RotateBy(units::degree_t(offSet)).Degrees().value();
  }  

  double getPID(double setPoint){
    return rotatorPID.Calculate(getAngle(), setPoint);
  }

  void setAngle(double angle){
    this->angle = angle;
  }


  frc::SwerveModuleState getState(){
    frc::SwerveModuleState state;

    state.angle = units::degree_t(getAngle());
    state.speed = units::meters_per_second_t(getSpeed());

    return state; 
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() {
    SetRotatorVoltage(getPID(angle));
  }

  void setPIDvalues(double kP, double kI, double kD, double f){
    rotatorPID.SetPID(kP, kI, kD);
    this->f = f;
  }

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX rotator;
  WPI_TalonFX wheel;

  CANCoder canCoder;
  frc2::PIDController rotatorPID{0.125, 0.5, 0};

  double angle = 0;
  double f = 0;

  double offSet;
};
