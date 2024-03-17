package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotIntakeSubsystem;



public class IntakePivotTeleop extends Command {

  PivotIntakeSubsystem pivotIntakeSubsystem;
  Supplier<Double> pivotSpeed;

  public IntakePivotTeleop(PivotIntakeSubsystem pivotIntakeSubsystem, Supplier<Double> pivotSpeed) {
    addRequirements(pivotIntakeSubsystem);
    this.pivotIntakeSubsystem = pivotIntakeSubsystem;
    this.pivotSpeed = pivotSpeed;
  }



// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shooterSubsystem.setStatus(ShooterSubsystemStatus.REVING);
    pivotIntakeSubsystem.returnIntakePivotEncoder().reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotIntakeSubsystem.setPivotMotor(pivotSpeed.get());
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
