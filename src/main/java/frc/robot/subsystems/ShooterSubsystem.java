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

  XboxController opcontrolly = new XboxController(1);

  double speedTarget;
  double getSpeedTop;
  double getSpeedBottom;
  double getRPMVelocity;


  double ampSpeedTop = -0.2;
  double ampSpeedBottom = -0.2;



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
    //shooterMotorBottomPIDController = shooterMotorBottom.getPIDController();

    shooterMotorTopPIDController.setP(0);
    shooterMotorTopPIDController.setI(0);
    shooterMotorTopPIDController.setD(0);
    shooterMotorTopPIDController.setFF(0.0017);
    shooterMotorTopPIDController.setOutputRange(-1, 1);


  }

  @Override
  public void periodic() {
    double topSpeed = SmartDashboard.getNumber("Shooter Top Speed", 0.2);
    double bottomSpeed = SmartDashboard.getNumber("Shooter Bottom Speed", 0.2);
    if((ampSpeedTop != topSpeed)) { 
      ampSpeedTop = topSpeed; 
    }
    if((ampSpeedBottom != bottomSpeed)) { 
      ampSpeedBottom = bottomSpeed; 
    }




    // --------------------------------------------//
    getSpeedBottom = shooterMotorTop.getBusVoltage() * shooterMotorTop.getAppliedOutput();
    getRPMVelocity = shooterTopEncoder.getVelocity();
    SmartDashboard.putNumber("Shooter Motor Voltage", getSpeedBottom);
    SmartDashboard.putNumber("Shooter Applied Output", shooterMotorTop.getAppliedOutput());
    SmartDashboard.putString("Shooter Status", shooterSubsystemStatus.toString());
    SmartDashboard.putNumber("Shooter Velocity RPM", shooterTopEncoder.getVelocity());

    if (getRPMVelocity >= 3100 ) {
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
      shooterMotorTopPIDController.setReference(-speed*5000 , ControlType.kVelocity); 
    //}
    SmartDashboard.putNumber("SetPoint", -speed*5000);

    //SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
  }

  public void setMotor(double shooterSpeed) {
    shooterMotorTop.set(shooterSpeed/1.5);
    shooterMotorBottom.set(shooterSpeed/1.5);
    SmartDashboard.putNumber("Shooter NUms", shooterSpeed/1.5);

  }

  public void ampSpeeds() {
    // shooterMotorTop.setVoltage(0.5);
    // shooterMotorBottom.setVoltage(0.45);

    shooterMotorTop.set(ampSpeedTop);
    shooterMotorBottom.set(ampSpeedBottom);

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
