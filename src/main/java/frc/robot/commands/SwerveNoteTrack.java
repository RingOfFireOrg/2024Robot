package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class SwerveNoteTrack extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    double speedDivide = 2;
    

    public SwerveNoteTrack(SwerveSubsystem swerveSubsystem){

        this.swerveSubsystem = swerveSubsystem;



        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        if (LimelightHelpers.getTV(Constants.VisionConstants.backCamera) == true) {
    
            double xSpeed = swerveSubsystem.noteTrackTranslationSpeed();
            //double ySpeed = ySpdFunctionRobot.get()/speedDivide; //idk how to move over the speed multipliers from SwerveJoystick to this
            double ySpeed = 0;
            double turningSpeed = swerveSubsystem.noteTrackRotSpeed();

            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed)
                    * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

            ChassisSpeeds chassisSpeeds;
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed); //robot centric

            // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            //     xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());  //Field Centric
            
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(moduleStates);
        }

    }



    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}