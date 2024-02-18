// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystemStatus;


public class IntakePivotTeleop extends Command {

  PivotIntakeSubsystem pivotIntakeSubsystem;
  Supplier<Double> pivotSpeed;

  public IntakePivotTeleop(PivotIntakeSubsystem pivotIntakeSubsystem, Supplier<Double> pivotSpeed) {
    addRequirements(pivotIntakeSubsystem);
    this.pivotIntakeSubsystem = pivotIntakeSubsystem;
    this.pivotSpeed = pivotSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
  }



// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shooterSubsystem.setStatus(ShooterSubsystemStatus.REVING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotIntakeSubsystem.setPivotMotor(pivotSpeed.get()/2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
