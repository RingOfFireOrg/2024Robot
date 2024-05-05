package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem.NoteSesnorStatus;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class SwerveNoteTrack extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private NoteSesnorStatus noteStatus;
    private boolean foundNote = false;
    

    public SwerveNoteTrack(SwerveSubsystem swerveSubsystem, NoteSesnorStatus noteStatus){
        this.swerveSubsystem = swerveSubsystem;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);


        this.noteStatus = noteStatus;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        SmartDashboard.putBoolean("LL_backCam Has Note", LimelightHelpers.getTV(Constants.VisionConstants.NoteCamera));    
        double ySpeed = 0;

        double turningSpeed = swerveSubsystem.noteTrackRotSpeed(Constants.VisionConstants.NoteRotationModifeier);
        //double turningSpeed = 0;
        
        double xSpeed = swerveSubsystem.noteTrackTranslationSpeed(Constants.VisionConstants.NoteTranslationModifier) - ((turningSpeed)/4);
        // //turningSpeed > 0.5 ? xSpeed = 0 : xSpeed;
        if (turningSpeed > 0.3) {
            xSpeed = 0;
        }
        if (Math.abs(xSpeed) > 0.01) {
            foundNote = true;
        }
        // if (foundNote == true && noteStatus != NoteSesnorStatus.NOTE_DECTECTED && xSpeed < 0.01) {
        //     xSpeed = 0.2;
        // }
        // else 
        //double xSpeed = 0;

        SmartDashboard.putNumber("LL_xSpeed", xSpeed);
        SmartDashboard.putNumber("LL_turnSpeed", turningSpeed);
        MathUtil.applyDeadband(turningSpeed, xSpeed);


        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed); //robot centric

        SmartDashboard.putNumber("LL_xSpeed post calc", xSpeed);
        SmartDashboard.putNumber("LL_turnSpeed post calc", turningSpeed);
        // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        //     xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());  //Field Centric
        
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
       

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