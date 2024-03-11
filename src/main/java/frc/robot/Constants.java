package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {

        public static final double kEncoderCPR = 2048;

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75; //L2
        public static final double kTurningMotorGearRatio = 1 / (150/7);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth =  Units.inchesToMeters(22.75); 
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22.75); 
        // Distance between front and back wheels


        //TODO: wrong order?
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
        );
        
        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;

        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10;

        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;

        public static final int kBackRightDriveMotorPort = 7;
        public static final int kBackRightTurningMotorPort = 8;
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;

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

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = (-1.721126444007689 ); //offset in radians - CanCoder 9
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = (0 - 1.768679848432144+ Math.PI); //offset in radians - CanCoder 10
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = (0 - -2.531068300011308+ Math.PI); //offset in radians - CanCoder 11
        //public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = (0); //offset in radians - CanCoder 11

        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = (0 - 0.490873852123405); //offset in radians - CanCoder 12

        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(15.1);
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

        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1;
        public static final int operatorSpareControllerPort = 2; //If we ever need a Second Operator Controler, use this

        //  Axis 
        public static final int leftStickX = 0;
        public static final int leftStickY = 1;
        public static final int rightStickX = 4;
        public static final int rightStickY = 5;

        public static final int leftTrigger = 2;  
        public static final int rightTrigger = 3; 

        // Buttons
        public static final int aButton = 1;
        public static final int bButton = 2;
        public static final int xButton = 3;
        public static final int yButton = 4;

        public static final int leftBumper = 5;
        public static final int rightBumper = 6;

        public static final int backButton = 7;
        public static final int startButton = 8;

        //Dpad ID and Angles

        public static final int DPadId = 0; //Controllers only have one POV
        
        public static final int dPadUp = 0;
        public static final int dPadUpRight = 45;
        public static final int dPadRight = 90;
        public static final int dPadDownRight = 135;
        public static final int dPadDown = 180;
        public static final int dPadDownLeft = 225;
        public static final int dPadLeft = 270;        
        public static final int dPadUpLeft = 315;




        

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

    public static final class LEDConstants {
        //Put any colors that are relevant

        public int ledLengthBar = 16+15+15;
        public int ledSegment1Start = 0; // Left bar
        public int ledSegment2Start = 16; // top Bar
        public int ledSegment3Start = 16+15; // Right Bar
        
        public int ledLegnthBoard = 0;



        public static int[] colorSet1 = {};
        public static int[] noteColor = {}; //Find a suitable orange
        
        public static int[] pyrotechRed = {}; //Find a suitable color
        public static int[] pyrotechOrange = {0,0,0}; //Find a suitable color

        int[] cryoTechBlue = {}; //Find a suitable color - // for blue alliance
        int[] cryotechTeal = {}; //Find a suitable color - // for blue alliance

        int[] loadingRed = {}; 
        int[] readyGreen = {}; 

        int[] boardColors = {};


    }

    public static final class IDConstants {
        //Swerve IDs
            // Not used, but here for consistancy 
        public static final int frontLeftDriveMotor = 1;
        public static final int frontLeftTurnMotor = 2;
        public static final int frontLeftCANcoder = 9;

        public static final int frontRightDriveMotor = 3;
        public static final int frontRightTurnMotor = 4;
        public static final int frontRightCANcoder = 10;

        public static final int backLeftDriveMotor = 5;
        public static final int backLeftDTurnMotor = 6;
        public static final int backLeftCANcoder = 11;

        public static final int backRightDriveMotor = 7;
        public static final int backRightTurnMotor = 8;
        public static final int backRightCANcoder = 12;

        // Attachment IDs
        public static final int intakeWheelMotorID = 21;  // Neo
        public static final int intakePivotMotorID = 22;  // ?
        public static final int shooterPivotMotorID = 23; // Talon
        public static final int shooterTopMotorID = 24;   // Neo
        public static final int shooterBottomMotorID = 25; // Neo
        public static final int leftClimberMotorID = 26; // Neo
        public static final int rightClimberMotorID = 27; //Neo
        


        // 21 - Intake Wheels
        // 22 - Intake Pivot
        // 23 - Shooter Pivot 
        // 24 - Shooter Wheel 1
        // 25 - Shooter Wheel 2
        // 26 - Left Climber
        // 27 - Right Climber
    }

    public enum ModulePosition {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }
}