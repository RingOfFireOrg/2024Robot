package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTeleop extends Command {

  IntakeSubsystem intakeSubsystem;
  Supplier<Double> intakeSpeed;


  public IntakeTeleop(IntakeSubsystem intakeSubsystem, Supplier<Double> intakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.intakeSpeed = intakeSpeed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    intakeSubsystem.setMotor(intakeSpeed.get());
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
