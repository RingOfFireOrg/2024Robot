// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KrakenShooterSubsystem;

public class KrakenShooterTeleop extends Command {
  KrakenShooterSubsystem krakenShooterSubsystem;
  Supplier<Double> speed;
  public KrakenShooterTeleop(KrakenShooterSubsystem krakenShooterSubsystem, Supplier<Double> speed) {
    addRequirements(krakenShooterSubsystem);
    this.krakenShooterSubsystem = krakenShooterSubsystem;
    this.speed = speed;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    krakenShooterSubsystem.setVelocity(speed.get());
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
