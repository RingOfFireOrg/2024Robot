// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotShooterSubsystem;

public class PivotShooterTeleop extends Command {


  PivotShooterSubsystem pivotSubsystem;
  Supplier<Double> pivotMovement;

  
  public PivotShooterTeleop(PivotShooterSubsystem pivotSubsystem, Supplier<Double> pivotMovement) {
    addRequirements(pivotSubsystem);
    this.pivotSubsystem = pivotSubsystem;
    this.pivotMovement = pivotMovement;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSubsystem.setMotor(pivotMovement.get());
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
