// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/Counter.h>
#include <frc/DigitalGlitchFilter.h>

class StorageAndDeliver : public frc2::SubsystemBase {
 public:
  StorageAndDeliver();
  void setIndexerVoltage(double voltage);
  void setFeederVoltage(double voltage);
  int getBallsShot();
  bool isTopSwitchPressed();
  bool isBottomSwitchPressed();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
    WPI_TalonSRX indexerMotor {5};
    WPI_VictorSPX lowerFeederMotor {6};
    //WPI_TalonSRX upperFeederMotor {9};

    frc::DigitalInput bottomLimit {0}, topLimit {1};
    frc::Counter ballCounter {&topLimit};
    frc::DigitalGlitchFilter digitalGlitchFilter;


  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
