// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMovementTimed extends Command {

  IntakeSubsystem intakeSubsystem;
  Timer time;
  MovementType movementType;

  public enum MovementType {
    UP,
    DOWN
  }

  public IntakeMovementTimed(IntakeSubsystem intakeSubsystem, MovementType movementType) {
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.movementType = movementType;
  }

  @Override
  public void initialize() {
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time.start();
    if ( movementType == MovementType.UP) {
      intakeSubsystem.setMotor(0.5);
    }
    else if (movementType == MovementType.DOWN) {
      intakeSubsystem.setMotor(-0.5);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (time.get() > 0.7) {
      intakeSubsystem.setMotor(0);
      return true;   
    }
    else {
    return false;
    }
  }
}
