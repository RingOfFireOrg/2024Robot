// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends Command {
  /** Creates a new LEDCommand. */
  LEDSubsystem ledSubsystem;
  String pattern;

  public LEDCommand(LEDSubsystem ledSubsystem, String pattern) {
    addRequirements(ledSubsystem);
    this.ledSubsystem = ledSubsystem;
    this.pattern = pattern;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ledSubsystem.setLed(pattern);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  } 

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}
