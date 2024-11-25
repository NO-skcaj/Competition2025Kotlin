// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutonomousLed.h"
#include "commands/SetLeds.h"

AutonomousLed::AutonomousLed(Leds *leds)
{
  // Use addRequirements() here to declare subsystem dependencies.

  // Set the command name
  SetName("AutonomusLed");

  AddCommands(
    SetLeds(LedMode::Rainbow,   5, leds),
    SetLeds(LedMode::Strobe,    5, leds),
    SetLeds(LedMode::HvaColors, 5, leds)
  );
}

