package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooterMotorTop; 
  private CANSparkMax shooterMotorBottom;
  private SparkPIDController shooterMotorTopPIDController;    // Delete if follow works
  private SparkPIDController shooterMotorBottomPIDController; // Delete if follow works
  //private SparkPIDController shooterMotorPIDController; 
  private RelativeEncoder shooterTopEncoder;
  private RelativeEncoder shooterBottomEncoder;

  double speedTarget;
  double getSpeedTop;
  double getSpeedBottom;
  double getRPMVelocityTop;
  double getRPMVelocityBottom;

  double ampSpeedTop = -0.2;
  double ampSpeedBottom = -0.2;
  double maxRPM = 5676;

  // move to constants
  double shooterMotorTopP;
  double shooterMotorTopI;
  double shooterMotorTopD;
  double shooterMotorTopFF;
  double shooterMotorTopMax;
  double shooterMotorTopMin;

  double shooterMotorBottomP;
  double shooterMotorBottomI;
  double shooterMotorBottomD;
  double shooterMotorBottomFF;
  double shooterMotorBottomMax;
  double shooterMotorBottomMin;


  public enum ShooterSubsystemStatus {
    READY,
    REVING,
    REVERSE,
    IDLE
  }

  ShooterSubsystemStatus shooterSubsystemStatus = ShooterSubsystemStatus.IDLE;

  public ShooterSubsystem() {
    shooterMotorTop    = new CANSparkMax(Constants.IDConstants.shooterTopMotorID,MotorType.kBrushless);
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



    // --------------------------------------------//
    //getSpeedBottom = shooterMotorTop.getBusVoltage() * shooterMotorTop.getAppliedOutput();
    getRPMVelocityTop = shooterTopEncoder.getVelocity();
    getRPMVelocityBottom = shooterBottomEncoder.getVelocity();
    SmartDashboard.putNumber("sShooter Motor Voltage", getSpeedBottom);
    //SmartDashboard.putNumber("sShooter Applied Output", shooterMotorTop.getAppliedOutput());
    SmartDashboard.putString("sShooter Status", shooterSubsystemStatus.toString());
    SmartDashboard.putNumber("sVelocity RPM TOP (periodic)", getRPMVelocityTop);
    SmartDashboard.putNumber("sVelocity RPM Bottom  (periodic)", getRPMVelocityBottom);


    if (getRPMVelocityTop >= 2800 ) {
      shooterSubsystemStatus = ShooterSubsystemStatus.READY;
    }
    else if (getRPMVelocityTop >= 100) {
      shooterSubsystemStatus = ShooterSubsystemStatus.REVING;
    }
    else if (getRPMVelocityTop <= -100) {
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
   // if (Math.abs(speed) > 0.1) { //add Deadband?
      shooterMotorTopPIDController.setReference( speed*maxRPM , ControlType.kVelocity); 
      shooterMotorBottomPIDController.setReference(speed*maxRPM , ControlType.kVelocity); 
    //}
    SmartDashboard.putNumber("sTargeted RPM", -speed*maxRPM);
    SmartDashboard.putNumber("sVelocity RPM TOP (refrence function)", shooterTopEncoder.getVelocity());
    SmartDashboard.putNumber("sVelocity RPM Bottom (refrence function))", shooterBottomEncoder.getVelocity());

  }

  public void setRefrenceRPM(double RPM){
    shooterMotorTopPIDController.setReference(RPM, ControlType.kVelocity); 
    shooterMotorBottomPIDController.setReference(RPM, ControlType.kVelocity); 
  }

  public void setMotor(double shooterSpeed) {
    shooterMotorTop.set(shooterSpeed/1.5);
    shooterMotorBottom.set(shooterSpeed/1.5);
  }

  public void ampSpeedsRaw() {
    // shooterMotorTop.setVoltage(0.5);
    // shooterMotorBottom.setVoltage(0.45);
    shooterMotorTop.set(ampSpeedTop);
    shooterMotorBottom.set(ampSpeedBottom);
  }

  public void ampSpeedsVelocityControl() {
    shooterMotorTopPIDController.setReference(-900, ControlType.kVelocity);
    shooterMotorBottomPIDController.setReference(-820 , ControlType.kVelocity); 
  }

  public CANSparkMax returnShooterMotorTop() {
    return shooterMotorTop;
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
