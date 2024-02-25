package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnToClimb extends Command {
  
  
  private SwerveSubsystem swerveSubsystem;
  private double currentAngle;
  private double goToAngle;
  private double rotationSpeed; 
  private double Angle1 = 60;
  private double Angle2 = 180;
  private double Angle3 = 300;

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
    if (Math.abs(currentAngle-Angle1) < Math.abs(currentAngle-Angle2) && Math.abs(currentAngle-Angle1) < Math.abs(currentAngle-Angle3)) {
      goToAngle = Angle1;
    }
    else if (Math.abs(currentAngle-Angle2) < Math.abs(currentAngle-Angle1) && Math.abs(currentAngle-Angle2) < Math.abs(currentAngle-Angle3)) {
      goToAngle = Angle2;
    }
    else if (Math.abs(currentAngle-Angle3) < Math.abs(currentAngle-Angle2) && Math.abs(currentAngle-Angle3) < Math.abs(currentAngle-Angle1)) {
      goToAngle = Angle3;
    }
    else {
      goToAngle = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = swerveSubsystem.getHeading();
    rotationSpeed = pidController.calculate(currentAngle, goToAngle);

    swerveSubsystem.drive(DriveConstants.kDriveKinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotationSpeed, swerveSubsystem.getPose().getRotation()))
    );
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
