// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <cmath>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>

class Hood : public frc2::SubsystemBase {
 public:
  Hood(){
    hoodMotor.SetSensorPhase(true);  
  }

  void VoltageMotor(double x){
    speed = x;
    }

    bool getSwitch(){
    if (!hoodSwitch.Get()){
      return true;
      }
      return false;
      }
    
  double getPosition(){
   return hoodMotor.GetSelectedSensorPosition();
  }

  /*hoodMotor.Config_kP(0, );
  hoodMotor.Config_kI();
  hoodMotor.Config_kD();*/

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override{
    
    target = 2135;
   distance = (target - getPosition());
   VoltageMotor(distance * 0.5) ;

   if (speed < 0 and getSwitch()){
     hoodMotor.SetVoltage(0_V);
   } else{
     hoodMotor.SetVoltage(units::volt_t(speed));
   }

   if (getSwitch()){
     hoodMotor.SetSelectedSensorPosition(0);
   }
  }


 private:
 frc::DigitalInput hoodSwitch {2};
 WPI_TalonSRX hoodMotor {9};
 double speed = 0;
 double target;
 double distance;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};


