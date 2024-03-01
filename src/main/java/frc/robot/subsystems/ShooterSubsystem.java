package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooterMotorTop; 
  private CANSparkMax shooterMotorBottom;
  private SparkPIDController shooterMotorTopPIDController;    // Delete if follow works
  private SparkPIDController shooterMotorBottomPIDController; // Delete if follow works
  private SparkPIDController shooterMotorPIDController; 
  private RelativeEncoder shooterTopEncoder;
  private RelativeEncoder shooterBottomEncoder;

  XboxController opcontrolly = new XboxController(1);

  double speedTarget;
  double getSpeedTop;
  double getSpeedBottom;
  double getRPMVelocity;
  double getRPMVelocityBelow;

  double ampSpeedTop = -0.2;
  double ampSpeedBottom = -0.2;
  double maxRPM = 5676;

  public enum ShooterSubsystemStatus {
    READY,
    REVING,
    REVERSE,
    IDLE
  }

  ShooterSubsystemStatus shooterSubsystemStatus = ShooterSubsystemStatus.IDLE;

  public ShooterSubsystem() {
    shooterMotorTop = new CANSparkMax(Constants.IDConstants.shooterTopMotorID,MotorType.kBrushless);
    shooterMotorBottom = new CANSparkMax(Constants.IDConstants.shooterBottomMotorID,MotorType.kBrushless);

    
    //shooterMotorBottom.follow(shooterMotorTop); // Idk if this implies they both will follow the same PID...

    shooterMotorTopPIDController = shooterMotorTop.getPIDController();
    shooterTopEncoder = shooterMotorTop.getEncoder();

    shooterMotorTopPIDController.setP(0.00015);
    shooterMotorTopPIDController.setI(0);
    shooterMotorTopPIDController.setD(0);
    shooterMotorTopPIDController.setFF(0.000155);
    shooterMotorTopPIDController.setOutputRange(-0.7, 1);

    shooterMotorBottomPIDController = shooterMotorBottom.getPIDController();
    shooterBottomEncoder = shooterMotorBottom.getEncoder();

    shooterMotorBottomPIDController.setP(0.00015);
    shooterMotorBottomPIDController.setI(0);
    shooterMotorBottomPIDController.setD(0);
    shooterMotorBottomPIDController.setFF(0.000155);
    shooterMotorBottomPIDController.setOutputRange(-0.7, 1);
  }

  @Override
  public void periodic() {

    //TODO: Remove for competition
    double topSpeed = SmartDashboard.getNumber("sShooter Top Speed", 0.2);
    double bottomSpeed = SmartDashboard.getNumber("sShooter Bottom Speed", 0.2);
    if((ampSpeedTop != topSpeed)) { 
      ampSpeedTop = topSpeed; 
    }
    if((ampSpeedBottom != bottomSpeed)) { 
      ampSpeedBottom = bottomSpeed; 
    }


    // --------------------------------------------//
    getSpeedBottom = shooterMotorTop.getBusVoltage() * shooterMotorTop.getAppliedOutput();
    getRPMVelocity = shooterTopEncoder.getVelocity() * -1;
    getRPMVelocityBelow = shooterBottomEncoder.getVelocity();
    SmartDashboard.putNumber("sShooter Motor Voltage", getSpeedBottom);
    SmartDashboard.putNumber("sShooter Applied Output", shooterMotorTop.getAppliedOutput());
    SmartDashboard.putString("sShooter Status", shooterSubsystemStatus.toString());
    SmartDashboard.putNumber("sShooter Velocity RPM TOP", getRPMVelocity);
    SmartDashboard.putNumber("sShooter Velocity RPM Bottom", getRPMVelocityBelow);


    if (getRPMVelocity >= 2800 ) {
      shooterSubsystemStatus = ShooterSubsystemStatus.READY;
    }
    else if (getRPMVelocity >= 100) {
      shooterSubsystemStatus = ShooterSubsystemStatus.REVING;
    }
    else if (getRPMVelocity <= -100) {
      shooterSubsystemStatus = ShooterSubsystemStatus.REVERSE;
    }
    else {  
      shooterSubsystemStatus = ShooterSubsystemStatus.IDLE;
    }
    
  }







  public ShooterSubsystemStatus getStatus() {
    return this.shooterSubsystemStatus;
  }

  public void setStatus(ShooterSubsystemStatus shooterSubsystemStatusSet) {
    shooterSubsystemStatus = shooterSubsystemStatusSet;
  }

  public void setRefrence(double speed){
   // if (Math.abs(speed) > 0.1) {

      shooterMotorTopPIDController.setReference( speed*maxRPM , ControlType.kVelocity); 
      shooterMotorBottomPIDController.setReference(speed*maxRPM , ControlType.kVelocity); 

    //}
    SmartDashboard.putNumber("sSpeed Input", speed);
    SmartDashboard.putNumber("sMaxrpm", maxRPM);
    SmartDashboard.putNumber("sSetPoint", -speed*maxRPM);

    SmartDashboard.putNumber("sTop Encoder Velocity", shooterTopEncoder.getVelocity());
    SmartDashboard.putNumber("sBottom Encoder Velocity", shooterBottomEncoder.getVelocity());

  }

  public void setMotor(double shooterSpeed) {
    shooterMotorTop.set(shooterSpeed/1.5);
    shooterMotorBottom.set(shooterSpeed/1.5);
    SmartDashboard.putNumber("sShooter Nums", shooterSpeed/1.5);

  }

  public void ampSpeedsRaw() {
    // shooterMotorTop.setVoltage(0.5);
    // shooterMotorBottom.setVoltage(0.45);
    shooterMotorTop.set(ampSpeedTop);
    shooterMotorBottom.set(ampSpeedBottom);
  }

  public void ampSpeedsVelocityControl() {
    SmartDashboard.putNumber("sAmp RPM", -750);
    shooterMotorTopPIDController.setReference(-750, ControlType.kVelocity);
    shooterMotorBottomPIDController.setReference(-750 , ControlType.kVelocity); 
 

  }

  public void setCoast(){
    shooterMotorTop.setIdleMode(IdleMode.kCoast);
    shooterMotorBottom.setIdleMode(IdleMode.kCoast);
  }
  public void setBrake(){
    shooterMotorTop.setIdleMode(IdleMode.kBrake);
    shooterMotorBottom.setIdleMode(IdleMode.kBrake);
  }
  public void setMotorStop() {
    shooterMotorTop.stopMotor();
    shooterMotorBottom.stopMotor();
  }
}
