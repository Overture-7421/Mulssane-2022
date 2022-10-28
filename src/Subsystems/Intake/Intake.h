#pragma once
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

class Intake : public frc2::SubsystemBase {
 public:
 Intake();
  
  //Function to initialize intakeMotor
  void invertIntakeMotor() {
    intakeMotor.SetInverted(true);
    };

  void moveIntakeMotor() {
    intakeMotor.SetVoltage(TalonSRXControlMode::Current, 6_V);
    intakeMotor.Set(TalonSRXControlMode::PercentOutput, 1);
    };

  void stopIntakeMotor() {
    intakeMotor.Set(TalonSRXControlMode::PercentOutput, 0.0);
    };
    
    //set doubleSolenoid Forward or Reverse

  void intakeSolenoidForward() {
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    };

  void intakeSolenoidReverse() {
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    };
    
  void Periodic() override;

 private:
 //Declare Talon and Solenoid of the Intake
 TalonSRX intakeMotor {8};
 frc::DoubleSolenoid intakeSolenoid {frc::PneumaticsModuleType::CTREPCM, 1, 0};
};