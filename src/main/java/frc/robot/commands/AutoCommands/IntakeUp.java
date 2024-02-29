// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotIntakeSubsystem;

public class IntakeUp extends Command {
  PivotIntakeSubsystem pivotIntakeSubsystem;
  public IntakeUp(PivotIntakeSubsystem pivotIntakeSubsystem) {
    addRequirements(pivotIntakeSubsystem);
    this.pivotIntakeSubsystem = pivotIntakeSubsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivotIntakeSubsystem.setPivotMotor(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotIntakeSubsystem.returnIntakePivotEncoder().getAbsolutePosition() < 0.2;
  }
}
