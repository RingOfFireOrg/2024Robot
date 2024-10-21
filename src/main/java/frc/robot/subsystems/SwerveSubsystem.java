
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
    
    private final AHRS gyro = new AHRS(SerialPort.Port.kOnboard);
    ShuffleboardTab generateAutoTab = Shuffleboard.getTab("Generate Auto");
    private Field2d field = new Field2d();
    private Field2d field2 = new Field2d();

    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, getRotation2d(),
        getSwerveModulePosition());

    private final PIDController yController = new PIDController(AutoConstants.kPYController, 0.0, 0.0);
    private final PIDController xController = new PIDController(AutoConstants.kPXController, 0.0, 0.0);
    private final PIDController thetaController = new PIDController(AutoConstants.kPThetaController,0.0, 0.0);

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );

    private boolean rejectVisionUpdate = false;
    private boolean rejectSwerveUpdate = false;



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
                new PIDConstants(AutoConstants.kTranslationP, AutoConstants.kTranslationI, AutoConstants.kTranslationD), // Translation PID constants
                new PIDConstants(AutoConstants.kRotationP, AutoConstants.kRotationI, AutoConstants.kRotationD), // Rotation PID constants
                AutoConstants.kMaxModuleSpeed, // Max module speed, in m/s
                AutoConstants.kDriveBaseRadius, /// Drive base radius in meters. Distance from robot center to furthest module.
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
            return true;        
        },
        this 
        );
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);
        generateAutoTab.add(field).withPosition(8, 0).withSize(9, 5);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        double temp = (gyro.getAngle() % 360);
        if(temp < 0) {
            temp = temp + 360; 
        }
        return temp;
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

    public void fieldCentricReset() {
        gyro.reset();
    }

    public AHRS getGyro() {
        return gyro;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),getSwerveModulePosition(),pose);
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

        field.setRobotPose(getPose());
        publisher.set(getModuleStates());

//        LimelightHelpers.SetRobotOrientation("limelight-tag", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-tag", gyro.getYaw(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-tag");
        
        field2.setRobotPose(mt2.pose);
        SmartDashboard.putData("Limelight PoseEst", field2);



        if(Math.abs(gyro.getRate()) > 720) 
        {
            rejectVisionUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            rejectVisionUpdate = true;
        }
        if(!rejectVisionUpdate)
        {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            m_poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }
        
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

    public void runModuleState(SwerveModuleState[] desiredStates) {
        frontLeft.runDesiredState(desiredStates[0]);
        frontRight.runDesiredState(desiredStates[1]);
        backLeft.runDesiredState(desiredStates[2]);
        backRight.runDesiredState(desiredStates[3]);
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


    /* ------ Note Tracking Functions ------- */

    public double noteTrackTranslationSpeed(double modifier) {
        SmartDashboard.putNumber("LL_TY", LimelightHelpers.getTY(Constants.VisionConstants.NoteCamera));
        return LimelightHelpers.getTY(Constants.VisionConstants.NoteCamera) / modifier;
    }

    public double noteTrackRotSpeed(double modifier) {
        return LimelightHelpers.getTX(Constants.VisionConstants.NoteCamera) / modifier;
    }

    public double tagStrafeLockTranslation(double modifier) {
        //SmartDashboard.putNumber("LL_TY", LimelightHelpers.getTY(Constants.VisionConstants.NoteCamera));
        return LimelightHelpers.getTX(Constants.VisionConstants.AprilTagCamera) / modifier;
    }

    public double tagStrafeLockRotation() {
        double tagID = LimelightHelpers.getFiducialID(Constants.VisionConstants.AprilTagCamera);
        double rotateTo = 360;
        if (tagID == 7); {
            rotateTo = 0;
        }
        return rotateTo;
        //LimelightHelpers.getTY(Constants.VisionConstants.AprilTagCamera) / modifier;
    }
  
    /* ------------------------------------- */
}
    
    