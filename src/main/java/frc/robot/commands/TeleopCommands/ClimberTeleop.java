// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTeleop extends Command {
  
  ClimberSubsystem climberSubsystem;
  Supplier<Double> climberSpeedSupplier;
  
  
  public ClimberTeleop(ClimberSubsystem climberSubsystem, Supplier<Double> climberSpeedSupplier) {
    addRequirements(climberSubsystem);
    this.climberSubsystem = climberSubsystem;

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climberSubsystem.setMotor(climberSpeedSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
