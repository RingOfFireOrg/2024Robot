package frc.robot.commands.TeleopCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnToClimb extends Command {
  
  
  private SwerveSubsystem swerveSubsystem;
  private Double currentAngle;

  private PIDController pidController = new PIDController(0,0,0);

  public TurnToClimb(SwerveSubsystem swerveSubsystem) {
    addRequirements(swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(1);
  }

  @Override
  public void initialize() {
    currentAngle = swerveSubsystem.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
