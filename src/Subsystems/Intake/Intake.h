#pragma once
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>

class Intake : public frc2::SubsystemBase {
 public:
  Intake() { intakeMotor.SetInverted(true); };

  // intake motor
  void setMotor(double voltage) {
    intakeMotor.SetVoltage(units::volt_t(voltage));
  }

  // intake solenoids
  void setPistons(bool state) {
    if (state) {
      intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
      intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
  }

  void Periodic() override;

 private:
  // Declare Talon and Solenoid of the Intake
  WPI_TalonSRX intakeMotor{8};
  frc::DoubleSolenoid intakeSolenoid{frc::PneumaticsModuleType::CTREPCM, 1, 0};
};