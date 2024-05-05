package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;  //DEPRECIATED 
import com.revrobotics.CANSparkLowLevel.MotorType;    //Non Depreciated

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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

    private final PIDController drivePIDController = new PIDController(
        2,
        0,
        0
    );
    private final ProfiledPIDController  turnPPIDController = new ProfiledPIDController (
        2,
        0,
        0,
        new TrapezoidProfile.Constraints(3 * Math.PI, 6 * Math.PI)

    );

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        0.1, 
        0, 
        0
    );

    private final SimpleMotorFeedforward azimuthFeedForward = new SimpleMotorFeedforward(
        0, 
        0, 
        0
    );

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
        turningMotor.setSmartCurrentLimit(60);

        driveMotor.burnFlash();
        turningMotor.burnFlash();
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

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        turnPPIDController.enableContinuousInput(-Math.PI, Math.PI);


        resetEncoders();
    }
    
    public double getDrivePosition() { 
       return (driveEncoder.getPosition() * ModuleConstants.kDriveMotorGearRatio) * Math.PI * ModuleConstants.kWheelDiameterMeters;
    }

    public double getTurningPosition() {
        return (getAbsoluteEncoderRad());
    }

    public double getDriveVelocity() {
        return ((driveEncoder.getPosition() / ModuleConstants.kEncoderCPR) * ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    }

    public double getAbsoluteEncoderRad() {


        double angle = absoluteEncoder.getAbsolutePosition().getValue() * (2*Math.PI);        
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

        SmartDashboard.putNumber("Swerve[" + encId + "] state", getAbsoluteEncoderRad());
        SmartDashboard.putNumber("degrees_Swerve[" + encId + "] state", Units.radiansToDegrees(getAbsoluteEncoderRad()));
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

    public void setDesiredStatePID(SwerveModuleState state) {
        //double angle = absoluteEncoder.getAbsolutePosition();
        double driveVelocity = getDriveVelocity();
        double turningPosition = getTurningPosition();

        SmartDashboard.putNumber("Swerve[" + encId + "] state", getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Module[" + encId + "]", state.angle.getDegrees());
        SmartDashboard.putNumber("swerve_[" + encId + "] Velocity", driveMotor.getEncoder().getVelocity());
   

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // state = SwerveModuleState.optimize(state, getState().angle);

        // driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        final double driveVoltage =
            drivePIDController.calculate(driveVelocity, state.speedMetersPerSecond)
                 + driveFeedforward.calculate(state.speedMetersPerSecond);

        final double turningVoltage =
            turnPPIDController.calculate(turningPosition, state.angle.getRadians())
                + azimuthFeedForward.calculate(turnPPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber("swerve_["+encId+"] driving voltage", driveVoltage);
        SmartDashboard.putNumber("swerve_["+encId+"] turning voltage", turningVoltage);
        driveMotor.setVoltage(driveVoltage);
        turningMotor.setVoltage(turningVoltage);
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

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
  }

    public double returnDriveMotorTemp() {
        return driveMotor.getMotorTemperature();
    }
}