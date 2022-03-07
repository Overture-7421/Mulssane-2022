// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetClimberPistonsDown.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "Commands/Common/SetIntake/SetIntake.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SetClimberPistonsDown::SetClimberPistonsDown(Climber* climber, Intake* intake) {
  AddCommands(
    frc2::InstantCommand([climber] { climber->setPistons(false); },
                                   {climber}),
    frc2::WaitCommand(1_s),
    SetIntake(intake, 0.0, false));
}
