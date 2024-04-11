package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotIntakeSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem.PivotSubsystemStatus;

public class IntakeDown extends Command {
  private PivotIntakeSubsystem pivotIntakeSubsystem;
  private double movementSpeed;
  public IntakeDown(PivotIntakeSubsystem pivotIntakeSubsystem, double movementSpeed) {
    addRequirements(pivotIntakeSubsystem);
    this.pivotIntakeSubsystem = pivotIntakeSubsystem;
    this.movementSpeed = movementSpeed;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if ( pivotIntakeSubsystem.getIntakeStatus() != PivotSubsystemStatus.INTAKE_DOWN) {
      pivotIntakeSubsystem.setPivotMotor(movementSpeed);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return pivotIntakeSubsystem.returnIntakePivotEncoder().getAbsolutePosition() > 0.7;
    return pivotIntakeSubsystem.getIntakeStatus() == PivotSubsystemStatus.INTAKE_DOWN;
  }
}
