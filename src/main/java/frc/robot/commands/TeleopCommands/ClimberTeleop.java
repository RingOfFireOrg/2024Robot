// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTeleop extends Command {
  
  ClimberSubsystem climberSubsystem;
  Supplier<Double> speed;
  XboxController oppyController = new XboxController(1);
  
  
  public ClimberTeleop(ClimberSubsystem climberSubsystem, Supplier<Double> speed) {
    addRequirements(climberSubsystem);
    this.climberSubsystem = climberSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberSubsystem.setMotor(speed.get()/1.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
