package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterTeleop extends Command {

  ShooterSubsystem shooterSubsystem;
  Supplier<Double> shooterSpeed;




  /*
   *  Outdated, don't use
   */

















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
