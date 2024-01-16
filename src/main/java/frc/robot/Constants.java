package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {

        public static final double kEncoderCPR = 2048;

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75; //L1
        public static final double kTurningMotorGearRatio = 1 / (150/7);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = 0.5334; //meters //Units.inchesToMeters(21); //TODO: Update with new chassis
        // Distance between right and left wheels
        public static final double kWheelBase = 0.69; //meters //Units.inchesToMeters(25.5); //TODO: Update with new chassis
        // Distance between front and back wheels

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
        );
        
        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;

        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 11;

        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 10;

        public static final int kBackRightDriveMotorPort = 7;
        public static final int kBackRightTurningMotorPort = 8;
        public static final int kBackRightDriveAbsoluteEncoderPort = 9;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = (-0.579844737820772 + -0.682621450609111); //offset in radians #12

        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = (4.485359823777614); //offset in radians #11

        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = (0); //offset in radians #10
        
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = (-5.008447272446618 + Math.PI); //offset in radians #9

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.267;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond ; //change denomenator
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4; //change denomenator
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }


    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI*0.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI/4;
        public static final double kPXController = 0.75; //multiplier for controller PID control
        public static final double kPYController = 0.75; //multiplier for controller PID control
        public static final double kPThetaController = 1.75; //multiplier for controller PID control

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {

        // we shoudl actually update the code to use all these 💀

        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1;
        public static final int operatorSpareControllerPort = 2; //If we ever need a Second Operator Controler, use this

        //  Axis 
        public static final int leftStickX = 0;
        public static final int leftStickY = 1;
        public static final int rightStickX = 4;
        public static final int rightStickY = 5;

        public static final int leftTrigger = 2;  //Returns Double
        public static final int rightTrigger = 3; //Returns Double

        // Buttons
        public static final int aButton = 1;
        public static final int bButton = 2;
        public static final int xButton = 3;
        public static final int yButton = 4;

        public static final int leftBumper = 5;
        public static final int rightBumper = 6;

        public static final int backButton = 7;
        public static final int startButton = 8;




        // ------Above are new for this season-----


        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 6;
        public static final int kAlignWithTargetButton = 5;
        public static final int kResetDirectionButton = 4;

        public static final int kRotate0Button = 3;
        public static final int kRotate180Button = 2;
        public static final int kExtendFullButton = 4;
        public static final int kRetractButton = 1;
        public static final int kToggleGrabButton = 10;
        public static final int kReverseGrabButton = 6;
        public static final int kForwardGrabButton = 5;
        public static final int kManuelButton = 7;

        public static final double kDeadband = 0.05;
    }

    public enum ModulePosition {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }
    public static final int LimitSwitchDIO = 1;
    public static final int WheeliePWM = 0;
}