package frc.robot.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;


public class PIDAutoBalancer extends Command {
    private final SwerveSubsystem drivetrainSubsystem;
    private final PIDController pidController;
    private final Timer timer;

    public PIDAutoBalancer(SwerveSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.pidController = new PIDController(
                /*AutoConstants.PITCH_P*/ 0.045,
                /*AutoConstants.PITCH_I*/0.0,
                /*AutoConstants.PITCH_D*/0.005
        );

        timer = new Timer();
        timer.start();
        this.pidController.setTolerance(/*AutoConstants.PITCH_TOLERANCE*/1.0);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        double val = -MathUtil.clamp(
                pidController.calculate(
                        drivetrainSubsystem.getPitchAsRotation2d().getDegrees(), 0), -.6, .6);

        if (Math.abs(drivetrainSubsystem.getPose().getRotation().getDegrees()) >= 90.0) {
            val = -val;
            SmartDashboard.putNumber("Auto Val", val);
        }
        drivetrainSubsystem.drive(DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(-val, 0.0, 0.0, drivetrainSubsystem.getPose().getRotation())));

    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(0.50)) {
            timer.reset();
            return Math.abs(drivetrainSubsystem.getPitchAsRotation2d().getDegrees()) <= 1.0;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}