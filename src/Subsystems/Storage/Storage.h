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

  void StoreBall() {Position1.Set(-.5);
                    Position2.Set(.5);
                    Omnis.Set(.5);    };
  
  void StopAll() {Position1.Set(0);
                    Position2.Set(0);
                    Omnis.Set(0);    }
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

WPI_TalonSRX Position2 {7};
WPI_VictorSPX Position1 {5};
WPI_VictorSPX Omnis {4};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};