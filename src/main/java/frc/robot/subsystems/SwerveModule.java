package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;  //DEPRECIATED 
import com.revrobotics.CANSparkLowLevel.MotorType;    //Non Depreciated

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder  driveEncoder;
    private final RelativeEncoder  turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private int encId;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        absoluteEncoder = new CANcoder(absoluteEncoderId);

        // driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        // turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        //testMotor = new CANSparkMax(absoluteEncoderId, null);

        driveMotor.setSmartCurrentLimit(60);
        driveMotor.burnFlash();
        //turningMotor.setSmartCurrentLimit(40);


        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();


        
        //set to coast;
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);


        //turningEncoder.setIntegratedSensorPosition(absoluteEncoder., timeoutMs)

        encId = absoluteEncoderId;



        //NEW CODE - Commented out configs. No longer needed(?)  instead you use rotations  and do unit conversions where needed

        // CANcoderConfiguration config = new CANcoderConfiguration();
        // // set units of the CANCoder to radians, with velocity being radians per second
        

        // config.sensorCoefficient = 2 * Math.PI / 4096.0; //convert to radians
        // config.unitString = "rad";
        // config.sensorTimeBase = SensorTimeBase.PerSecond;

        // //absoluteEncoder.configAllSettings(config);
        // //absoluteEncoder.getConfigurator().apply(config)
        

        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        ///driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);


        resetEncoders();
    }
    
    public double getDrivePosition() { //math is for manual conversion factor because TalonFX controllers do not have ConversionFactor functions
       return (driveEncoder.getPosition()* ModuleConstants.kDriveMotorGearRatio) * Math.PI * ModuleConstants.kWheelDiameterMeters;
    }


 

    public double getTurningPosition() {
        return (getAbsoluteEncoderRad());
    }

    public double getDriveVelocity() {
        return ((driveEncoder.getPosition() / ModuleConstants.kEncoderCPR) * ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    }



    //     THIS is throwing an error due to an API change, but I dont think we actually need it (?) its not used anywhere else so Im commenting it out
    // public double getTurningVelocity() {
    //     return (absoluteEncoder.getVelocity());
    // }

    public double getAbsoluteEncoderRad() {

        //NEW CODE add get value to convert getAbsPos status signal to a double
        // Getting abs pos may now be in rotations, need to convert to radians(?)
        double angle = absoluteEncoder.getAbsolutePosition().getValue() * (2*Math.PI);
        //angle = Math.toRadians(angle);
        
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        //double angle = absoluteEncoder.getAbsolutePosition();

        SmartDashboard.putNumber("Swerve[" + encId + "] state", getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Module[" + encId + "]", state.angle.getDegrees());
        SmartDashboard.putNumber("swerve_[" + encId + "] Velocity", driveMotor.getEncoder().getVelocity());
   

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void brake(boolean doBrake){
        if(doBrake){
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }
        else{
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }
        
    }


    public double returnDriveMotorTemp() {
        return driveMotor.getMotorTemperature();
    }
}