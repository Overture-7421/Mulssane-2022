// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetClimberPistonsUp.h"
#include "Commands/Common/SetIntake/SetIntake.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>
// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SetClimberPistonsUp::SetClimberPistonsUp(Intake* intake, Climber* climber) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());

  AddCommands(
    SetIntake(intake, 0, true),
    frc2::WaitCommand(0.5_s),
    frc2::InstantCommand([climber]{climber->setPistons(true);}, {climber})
  );
}
