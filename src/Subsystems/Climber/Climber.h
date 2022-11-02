#pragma once
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>


class Climber : public frc2::SubsystemBase {
 public:
 Climber();
  
  //Functions to initialize rightclimberMotor
  void invertRightClimberMotor() {
    rightClimberMotor.SetInverted(true);
    };

  void moveRightClimberMotor() {
    rightClimberMotor.SetVoltage(units::volt_t (6));
    };

  void stopRightClimberMotor() {
    rightClimberMotor.SetVoltage(units::volt_t (0));
    };

  //Functions to initialize leftclimberMotor
  void invertLeftClimberMotor() {
    leftClimberMotor.SetInverted(true);
    };

  void moveLeftClimberMotor() {
    leftClimberMotor.SetVoltage(units::volt_t (6));
    };

  void stopLeftClimberMotor() {
    leftClimberMotor.SetVoltage(units::volt_t (0));
    };
  
    
    //set doubleSolenoid Forward or Reverse

  void climberSolenoidForward() {
    climberSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    };

  void climberSolenoidReverse() {
    climberSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    };
    
  void Periodic() override;

 private:
 //Declare Talons and Solenoid of the Intake
 WPI_TalonSRX rightClimberMotor {10};
 WPI_TalonSRX leftClimberMotor {6};
 frc::DoubleSolenoid climberSolenoid {frc::PneumaticsModuleType::CTREPCM, 1, 0};
};