package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem.PivotSubsystemStatus;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class SwerveNoteTrackFull extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final PivotIntakeSubsystem pivotIntakeSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    

    public SwerveNoteTrackFull(
            SwerveSubsystem swerveSubsystem, 
            PivotIntakeSubsystem pivotIntakeSubsystem,
            IntakeSubsystem intakeSubsystem
        ){

        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.pivotIntakeSubsystem = pivotIntakeSubsystem;



        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
        addRequirements(pivotIntakeSubsystem);
        addRequirements(intakeSubsystem);


    }

    @Override
    public void initialize() {

        
    }

    @Override
    public void execute() {
        intakeSubsystem.setMotorFull(0.7);

            if (LimelightHelpers.getTV(Constants.VisionConstants.NoteCamera) == true) {
        
                double xSpeed = swerveSubsystem.noteTrackTranslationSpeed(10);
                double ySpeed = 0;
                double turningSpeed = swerveSubsystem.noteTrackRotSpeed(30);

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
        intakeSubsystem.stopIntakeWheel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}