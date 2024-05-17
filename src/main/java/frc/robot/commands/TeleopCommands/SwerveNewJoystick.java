package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveNewJoystick extends Command {

    private final SwerveSubsystem swerveSubsystem;

    private final Supplier<Double> xSpdFunctionField, ySpdFunctionField, xSpdFunctionRobot, ySpdFunctionRobot, turningSpdFunctionLeft, turningSpdFunctionRight;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final SlewRateLimiter xLimiterOLD, yLimiterOLD, turningLimiterOLD;
    private final XboxController driveController = new XboxController(0);


    public SwerveNewJoystick(SwerveSubsystem swerveSubsystem, 
      Supplier<Double> xSpdFunctionField, 
      Supplier<Double> ySpdFunctionField, 

      Supplier<Double> xSpdFunctionRobot,
      Supplier<Double> ySpdFunctionRobot,

      Supplier<Double> turningSpdFunctionLeft,
      Supplier<Double> turningSpdFunctionRight,
      Supplier<Boolean> fieldOrientedFunction

      ) 
      {

        this.swerveSubsystem = swerveSubsystem;

        this.xSpdFunctionField = xSpdFunctionField;
        this.ySpdFunctionField = ySpdFunctionField;

        this.xSpdFunctionRobot = xSpdFunctionRobot;
        this.ySpdFunctionRobot = ySpdFunctionRobot;

        this.turningSpdFunctionLeft = turningSpdFunctionLeft;
        this.turningSpdFunctionRight = turningSpdFunctionRight;


        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.xLimiterOLD = new SlewRateLimiter(6.5);
        this.yLimiterOLD = new SlewRateLimiter(6.5);
        this.turningLimiterOLD = new SlewRateLimiter(5);
                
        addRequirements(swerveSubsystem);

    }

    @Override
    public void initialize() {
        
    }

    // public double signFunc(double input) {
    //   val1 = false;
    //   if
    //   return (input > 0) - (input < 0);
    // }

    @Override
    public void execute() {
      if(driveController.getRawButton(7) == true) {
        swerveSubsystem.fieldCentricReset();
    }

      if (Math.abs(xSpdFunctionField.get()) >= 0.05  || Math.abs(ySpdFunctionField.get()) >= 0.05) 
      {
        double xSpeed = xSpdFunctionField.get(); 
        xSpeed = (1 / (1 - SwerveConstants.kDeadband)) * (xSpeed + ( -Math.signum(xSpeed) * SwerveConstants.kDeadband));
        double ySpeed = ySpdFunctionField.get();
        double thetaSpeed = turningSpdFunctionLeft.get() - turningSpdFunctionRight.get();

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        thetaSpeed = turningLimiter.calculate(thetaSpeed);

        double linearMagnitude = Math.pow(MathUtil.applyDeadband(Math.hypot(xSpeed, ySpeed), SwerveConstants.kDeadband),2);
        //linearMagnitude = linearMagnitude * linearMagnitude;
        Rotation2d linearDirection = new Rotation2d(xSpeed, ySpeed);
        thetaSpeed = Math.abs(thetaSpeed) > SwerveConstants.kDeadband ? thetaSpeed : 0.0;
        thetaSpeed = Math.copySign(thetaSpeed * thetaSpeed, thetaSpeed);

        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
          .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
          .getTranslation();

        // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        //   linearVelocity.getX() * SwerveConstants.kMaxSpeed, 
        //   linearVelocity.getY() * SwerveConstants.kMaxSpeed,
        //   thetaSpeed * SwerveConstants.kMaxAngularSpeed, 
        //   swerveSubsystem.getRotation2d()
        // );


        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
          linearVelocity.getX() * SwerveConstants.kMaxSpeed, 
          linearVelocity.getY() * SwerveConstants.kMaxSpeed, 
          thetaSpeed * SwerveConstants.kMaxAngularSpeed);

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeed);

        swerveSubsystem.setModuleStates(moduleStates);
        //swerveSubsystem.setModuleStates(moduleStates);
      }
      else if (xSpdFunctionRobot.get() >= 0.1 || xSpdFunctionRobot.get() <= -0.1 || ySpdFunctionRobot.get() >= 0.1 || ySpdFunctionRobot.get() <= -0.1)
      {

        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunctionRobot.get();
        double ySpeed = ySpdFunctionRobot.get();
        //double ySpeed = 0;

        double turningSpeed = turningSpdFunctionLeft.get() - turningSpdFunctionRight.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiterOLD.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiterOLD.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);

      }
    else {
        /* Joystick Input */
        //double xSpeed = xSpdFunctionRobot.get()/speedDivide;
        //double ySpeed = ySpdFunctionRobot.get()/speedDivide;
        double xSpeed = 0;
        double ySpeed = 0;
        double turningSpeed = turningSpdFunctionLeft.get() - turningSpdFunctionRight.get();

        /* Deadband */
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        /* Ratelimiter/Acceleration Limit */
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        /* Create Chassis speeds for each module */
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        /* Output Chassis Speeds */
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