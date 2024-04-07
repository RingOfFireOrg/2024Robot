
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule
    (
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule frontRight = new SwerveModule
    (
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backLeft = new SwerveModule
    (
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backRight = new SwerveModule
    (
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed
    );
    
    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);
    ShuffleboardTab generateAutoTab = Shuffleboard.getTab("Generate Auto");
    private Field2d field = new Field2d();
    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, getRotation2d(),
        getSwerveModulePosition());

    private final PIDController yController = new PIDController(AutoConstants.kPYController, 0.0, 0.0);
    private final PIDController xController = new PIDController(AutoConstants.kPXController, 0.0, 0.0);
    private final PIDController thetaController = new PIDController(AutoConstants.kPThetaController,0.0, 0.0);

    public SwerveSubsystem() {

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(9, 0.0, 0.5), // Translation PID constants
                new PIDConstants(6, 0.0, 0.5), // Rotation PID constants
                4.5, // Max module speed, in m/s
                Units.inchesToMeters(15.909905), // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() 
        ),
        () -> {

            // var alliance = DriverStation.getAlliance();
            //     if (alliance.isPresent()) {
            //         return alliance.get() == DriverStation.Alliance.Red;
            //     }
            // return false;
            var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Blue;
                }
            //return alliance.get() == DriverStation.Alliance.Red;    
            return true;        
        },
        this // Reference to this subsystem to set requirements
        );
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);
        generateAutoTab.add(field).withPosition(8, 0).withSize(9, 5);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    // public double getHeading() {
    //     if(gyro.isMoving() == true) {
    //         SmartDashboard.putNumber("Gyro Angle",gyro.getAngle());
    //     }
    //     return gyro.getAngle();
    //     //return Math.IEEEremainder(gyro.getAngle(), 360);

    // }
    public double getHeading() {
        //double temp = Math.IEEEremainder(gyro.getAngle(), 360);
        SmartDashboard.putNumber("swerve_Direct Get Angle", gyro.getAngle());

        double temp = (gyro.getAngle() % 360);
        if(temp < 0) {
            temp = temp + 360; 
        }
        return temp;
        // should be the same as: return (gyro.getAngle() % 360);

    }



    // ---------------------------------------------------------------------------------------------------------------
    public Pose2d getPose() {
        SmartDashboard.putString( "swerve_Get Pose meters ", odometer.getPoseMeters().toString()); 
        return odometer.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        //odometer.resetPosition(getRotation2d(),getSwerveModulePosition(),pose);
        odometer.resetPosition(new Rotation2d(Math.toRadians(gyro.getYaw())),getSwerveModulePosition(),pose); 
        
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
       // return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()); 
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(moduleStates);
    }

    public void driveRobotRelative2(ChassisSpeeds speeds) { 
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        setModuleStates(swerveModuleStates);

    }

    // ---------------------------------------------------------------------------------------------------------------

    // public double gyroangle() {
    //     while(gyro.isMoving()) {
    //         return gyro.getAngle() % 360;
    //     }
    //     return gyro.getAngle();
    // }
    // public void setModuleStates(SwerveModuleState[] desiredStates) {
    //     SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    //     frontLeft.setDesiredState(desiredStates[0]);
    //     frontRight.setDesiredState(desiredStates[1]);
    //     backLeft.setDesiredState(desiredStates[2]);
    //     backRight.setDesiredState(desiredStates[3]);
    // }

    public void fieldCentricReset() {
        gyro.reset();
    }
    public double pitchVals() {
        return gyro.getPitch();
    }
    public double yawVals() {
        return gyro.getYaw();
    }
    public double rollVals() {
        return gyro.getRoll();
    }
    public Rotation2d getRotation2d() {
        //SmartDashboard.putNumber("R2d robot heading",  Rotation2d.fromDegrees(getHeading()).getDegrees());
        return Rotation2d.fromDegrees(getHeading());
    }

    public double getRotation2dButaDouble() {

        return getHeading();
    }



    void setModuleStates2(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
  

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),getSwerveModulePosition(),pose);
    }

    public boolean resetOdometry2(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),getSwerveModulePosition(),pose);
        return true;
    }

    public PIDController getxController() {
        return xController;
    }

    public PIDController getyController() {
        return yController;
    }

    public PIDController getThetaController() {
        return thetaController;
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getSwerveModulePosition());
        
        SmartDashboard.putNumber("swerve_Robot Heading", getHeading());
        SmartDashboard.putNumber("swerve_Robot Theta", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("swerve_FL Robot Tranlsation", frontLeft.getDrivePosition());
        SmartDashboard.putNumber("swerve_FR Robot Tranlsation", frontRight.getDrivePosition());
        SmartDashboard.putNumber("swerve_BL Robot Tranlsation", backLeft.getDrivePosition());
        SmartDashboard.putNumber("swerve_BR Robot Tranlsation", backRight.getDrivePosition());

        SmartDashboard.putNumber("swerve_FL Motor Temp", frontLeft.returnDriveMotorTemp());
        SmartDashboard.putNumber("swerve_FR Motor Temp", frontRight.returnDriveMotorTemp());
        SmartDashboard.putNumber("swerve_BL Motor Temp", backLeft.returnDriveMotorTemp());
        SmartDashboard.putNumber("swerve_BR Motor Temp", backRight.returnDriveMotorTemp());

        //SmartDashboard.putString("Swerve Current Command", this.getCurrentCommand().toString());
        field.setRobotPose(getPose());
        publisher.set(getModuleStates());
        
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModulePosition[] getSwerveModulePosition() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition())),
        };
    }


    public SwerveModuleState[] getModuleStates() { 
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()

        };


        // return new SwerveModuleState[] {
        //     new SwerveModuleState(frontLeft.getDrivePosition(), null),
        //     new SwerveModuleState(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
        //     new SwerveModuleState(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
        //     new SwerveModuleState(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition())),
        // };
    }    


    public void driveForward(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }


    public void brake(boolean doBrake){
        if(doBrake){
            frontLeft.brake(true);
            frontRight.brake(true);
            backLeft.brake(true);
            backRight.brake(true);
        }
        else{
            frontLeft.brake(false);
            frontRight.brake(false);
            backLeft.brake(false);
            backRight.brake(false);
        }
    }

    public void drive(SwerveModuleState... desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public Command resetGyro() {
        return this.runOnce(() ->  this.zeroHeading());
    }


    /* ------------- */

    public double noteTrackTranslationSpeed(double modifier) {
        SmartDashboard.putNumber("LL_TY", LimelightHelpers.getTY(Constants.VisionConstants.NoteCamera));
        return LimelightHelpers.getTY(Constants.VisionConstants.NoteCamera) / modifier;
    }

    public double noteTrackRotSpeed(double modifier) {
        return LimelightHelpers.getTX(Constants.VisionConstants.NoteCamera) / modifier;
    }

  
    /* ------------- */
}
    
    