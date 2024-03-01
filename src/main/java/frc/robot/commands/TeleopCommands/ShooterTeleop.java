// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystemStatus;


public class ShooterTeleop extends Command {

  ShooterSubsystem shooterSubsystem;
  Supplier<Double> shooterSpeed;

  public ShooterTeleop(ShooterSubsystem shooterSubsystem, Supplier<Double> shooterSpeed) {
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
    this.shooterSpeed = shooterSpeed;
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
    //shooterSubsystem.setMotor(shooterSpeed.get());
    shooterSubsystem.setRefrence(shooterSpeed.get()/1.65);
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
