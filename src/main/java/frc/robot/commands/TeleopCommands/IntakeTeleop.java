// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTeleop extends Command {

  IntakeSubsystem intakeSubsystem;
  double pivotSpeed;
  double wheelSpeed;
  private final XboxController opController = new XboxController(1);

  public IntakeTeleop(IntakeSubsystem intakeSubsystem, double pivotSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSpeed = pivotSpeed;
    //this.wheelSpeed = wheelSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //intakeSubsystem.setMotors(pivotSpeed, wheelSpeed);
    intakeSubsystem.setWheelMotor(opController.getLeftY());
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
