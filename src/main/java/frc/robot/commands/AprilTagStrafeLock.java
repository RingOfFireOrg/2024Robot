package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class AprilTagStrafeLock extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final Supplier<Double> leftAxis, rightAxis;
  ShuffleboardTab visionTab = Shuffleboard.getTab("Vision Tab");
  private PIDController pidController = new PIDController(0.001, 0, 0.3);

  public AprilTagStrafeLock(SwerveSubsystem swerveSubsystem, Supplier<Double> leftAxis, Supplier<Double> rightAxis) {
    this.swerveSubsystem = swerveSubsystem;
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    this.leftAxis = leftAxis;
    this.rightAxis = rightAxis;
    
    pidController.enableContinuousInput(0, 360);
    pidController.setTolerance(0.3);

    addRequirements(swerveSubsystem);  
  }


  @Override
  public void initialize() {
    pidController.reset();
    //double desiredAngle = swerveSubsystem.tagStrafeLockRotation(); 
    // double desiredAngle = swerveSubsystem.tagStrafeLockRotation(); 
    // if (desiredAngle != 360) {
    //   pidController.setSetpoint(desiredAngle);
    // }  
    // else {
    //   pidController.setSetpoint(0);
    // }
    

  }

  @Override
  public void execute() {
    
    double desiredAngle = swerveSubsystem.tagStrafeLockRotation(); 
    if (desiredAngle != 360) {
      pidController.setSetpoint(0);
    }  
    else {
      pidController.setSetpoint(0);
    }


    double xSpeed =  leftAxis.get() + rightAxis.get();  // Based on Controller //TODO: put a deadband
    double ySpeed = -swerveSubsystem.tagStrafeLockTranslation(30);  // Strafe to Center

    double currentAngle = swerveSubsystem.getHeading();
    double turningSpeed = MathUtil.applyDeadband(pidController.calculate(currentAngle),0.05);    

    // double desiredAngle = swerveSubsystem.tagStrafeLockRotation(); 
    
    // if (desiredAngle != 360) {
    //   pidController.setSetpoint(desiredAngle);
    // }  




    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed)
            * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed); //robot centric

    //visionTab.addNumber(getName(), null);
    SmartDashboard.putNumber("Tag Strafe Lock (YSpeed)", ySpeed);
    SmartDashboard.putNumber("Tag Strafe Lock (Rot Speed)", turningSpeed);

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
    
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
