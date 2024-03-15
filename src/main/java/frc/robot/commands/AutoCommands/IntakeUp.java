// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotIntakeSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem.PivotSubsystemStatus;

public class IntakeUp extends Command {
  PivotIntakeSubsystem pivotIntakeSubsystem;
  double movementSpeed;
  public IntakeUp(PivotIntakeSubsystem pivotIntakeSubsystem, double movementSpeed) {
    addRequirements(pivotIntakeSubsystem);
    this.pivotIntakeSubsystem = pivotIntakeSubsystem;
    this.movementSpeed = movementSpeed;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (pivotIntakeSubsystem.getIntakeStatus() != PivotSubsystemStatus.INTAKE_UP) {
      pivotIntakeSubsystem.setPivotMotor(-movementSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotIntakeSubsystem.getIntakeStatus() == PivotSubsystemStatus.INTAKE_UP;
  }
}
