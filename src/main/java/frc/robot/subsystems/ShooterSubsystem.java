package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {



  /*
    This Subsystem will be here just in case we switch back to neos or need it for refrence, 
    but the motor controllers have been switched to talonfx, so KrakenShooterSubsystem is the most up to date one
  */


  private CANSparkMax shooterMotorTop; 
  private CANSparkMax shooterMotorBottom;
  private SparkPIDController shooterMotorTopPIDController;    
  private SparkPIDController shooterMotorBottomPIDController; 
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

  public enum ShooterSubsystemStatus {
    READY,
    REVING,
    REVERSE,
    IDLE
  }

  ShooterSubsystemStatus shooterSubsystemStatus = ShooterSubsystemStatus.IDLE;

  public ShooterSubsystem() {
    // shooterMotorTop    = new CANSparkMax(Constants.IDConstants.shooterTopMotorID,MotorType.kBrushless);
    // shooterMotorBottom = new CANSparkMax(Constants.IDConstants.shooterBottomMotorID,MotorType.kBrushless);
    shooterMotorTop    = new CANSparkMax(24,MotorType.kBrushless);
    shooterMotorBottom = new CANSparkMax(25,MotorType.kBrushless);

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



    //getSpeedBottom = shooterMotorTop.getBusVoltage() * shooterMotorTop.getAppliedOutput();
    getRPMVelocityTop = shooterTopEncoder.getVelocity();
    getRPMVelocityBottom = shooterBottomEncoder.getVelocity();
    SmartDashboard.putNumber("sShooter Motor Voltage", getSpeedBottom);
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
    shooterMotorTopPIDController.setReference( speed*maxRPM , ControlType.kVelocity); 
    shooterMotorBottomPIDController.setReference(speed*maxRPM , ControlType.kVelocity); 
    SmartDashboard.putNumber("sTargeted RPM", -speed*maxRPM);
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
