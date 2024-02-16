// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem;

public class PivotIntakeTeleop extends Command {

  PivotIntakeSubsystem pivotIntakeSubsystem;
  Supplier<Double> pivotIntakeSpeed;

  public PivotIntakeTeleop(PivotIntakeSubsystem pivotIntakeSubsystem, Supplier<Double> pivotIntakeSpeed) {
    addRequirements(pivotIntakeSubsystem);
    this.pivotIntakeSubsystem = pivotIntakeSubsystem;
    this.pivotIntakeSpeed = pivotIntakeSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotIntakeSubsystem.setPivotMotor(pivotIntakeSpeed.get());
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
