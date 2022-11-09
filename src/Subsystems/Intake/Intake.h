#pragma once
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>


class Intake : public frc2::SubsystemBase {
 public:
 Intake();
  
  //Function to initialize intakeMotor
  void invertIntakeMotor() {
    intakeMotor.SetInverted(true);
    };

  void initializeMotor() {
    intakeMotor.Set(TalonSRXControlMode::PercentOutput, 1); //Pending to define speed...
    };

  void desinitializeMotor() {
    intakeMotor.Set(TalonSRXControlMode::PercentOutput, 0.0); //Pending to define speed...
    };
    
    //set doubleSolenoid Off, Forward or Reverse
  void intakeSolenoidForward() {
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    };

  void intakeSolenoidReverse() {
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    };
    
  void Periodic() override;

  

 private:
 //Declare Talon and Solenoid of the Intake
 WPI_TalonSRX intakeMotor {8};
 frc::DoubleSolenoid intakeSolenoid {frc::PneumaticsModuleType::CTREPCM, 1, 0};
};