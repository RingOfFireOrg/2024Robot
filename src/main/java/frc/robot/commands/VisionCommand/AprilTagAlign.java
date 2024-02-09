package frc.robot.commands.VisionCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AprilTagAlign extends Command {

  SwerveSubsystem swerveSubsystem;
  LimeLightSubsystem limeLightSubsystem;
  boolean targetCheck;
  double robotHeading;
  double setpoint;
  double rotationSpeed;
  double yaw = 0;
  
  PIDController thetaController = new PIDController(0.5, 0.0, 0.1); //TODO: Tune this 

  public AprilTagAlign(SwerveSubsystem swerveSubsystem, LimeLightSubsystem limeLightSubsystem) {
    addRequirements(swerveSubsystem, limeLightSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    this.limeLightSubsystem = limeLightSubsystem;

    thetaController.enableContinuousInput(-180, 180);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var result = limeLightSubsystem.getCamera().getLatestResult();
    targetCheck = result.hasTargets();

    if (targetCheck == false || result.getBestTarget().getPoseAmbiguity() >= 0.2) {
      System.out.println("No targets found aim the camera better bud");
      this.cancel(); // 😢
    } 
    else {
      double yawRadians = result.getBestTarget().getBestCameraToTarget().getRotation().getZ();
      yaw = Units.radiansToDegrees(yawRadians);
      setpoint = yaw - 180;
      //swerveBase.getNavX().reset(); ?
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotHeading = swerveSubsystem.getHeading();
    rotationSpeed = thetaController.calculate(robotHeading, setpoint);
    SmartDashboard.putNumber("Angle Rot Speed", rotationSpeed);

    swerveSubsystem.drive(DriveConstants.kDriveKinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotationSpeed, swerveSubsystem.getPose().getRotation())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(rotationSpeed) < 0.1;  
  }
}
