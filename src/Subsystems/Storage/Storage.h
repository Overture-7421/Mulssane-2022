// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

class Storage : public frc2::SubsystemBase {
 public:
  Storage();

  void StoreBall() {
    
  Position1.Set(-0.5);
  Position2.Set(0.5);
  Omnis.Set(0.5);    
  
  };
  
  void StopAll() {
    Position1.Set(0);
    Position2.Set(0);
    Omnis.Set(0);   
   }

  void Detect() {
  if(!input.Get(5)) {
        std::cout << 1;
    } else {
       std::cout << 0;
    }
  }
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

WPI_VictorSPX Position2 {7};
WPI_TalonSRX Position1 {5};
WPI_VictorSPX Omnis {4};
WPI_VictorSPX Feeder {10};
frc::DigitalInput input{5};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
}
