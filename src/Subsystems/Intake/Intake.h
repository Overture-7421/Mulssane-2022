#pragma once
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>

class Intake : public frc2::SubsystemBase {
 public:
 Intake();
  
  //Function to initialize intakeMotor
  void initializeMotor() {
    intakeMotor.Set(TalonSRXControlMode::PercentOutput, 0.5); //Pending to define speed...
    };
    
    //set doubleSolenoid Off, Forward or Reverse
  void solenoidOff() {
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    }; 

  void solenoidForward() {
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    };

  void solenoidReverse() {
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    };
    
    void Periodic() override;

 private:
 //Declare Talon and Solenoid of the Intake
 TalonSRX intakeMotor {9};
 frc::DoubleSolenoid intakeSolenoid {frc::PneumaticsModuleType::CTREPCM, 1, 2};
};