package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooterMotorTop; 
  private CANSparkMax shooterMotorBottom;
  private SparkPIDController shooterMotorTopPIDController;    // Delete if follow works
  private SparkPIDController shooterMotorBottomPIDController; // Delete if follow works
  private SparkPIDController shooterMotorPIDController; 

  double speedTarget;


  public enum ShooterSubsystemStatus {
    READY,
    REVING,
    IDLE
  }

  public ShooterSubsystemStatus shooterSubsystemStatus = ShooterSubsystemStatus.IDLE;

  public ShooterSubsystem() {
    shooterMotorTop = new CANSparkMax(Constants.IDConstants.shooterTopMotorID,MotorType.kBrushless);
    shooterMotorBottom = new CANSparkMax(Constants.IDConstants.shooterBottomMotorID,MotorType.kBrushless);
    
    shooterMotorBottom.follow(shooterMotorTop); // Idk if this implies they both will follow the same PID...

    shooterMotorPIDController = shooterMotorTop.getPIDController();
    //shooterMotorBottomPIDController = shooterMotorBottom.getPIDController();

    shooterMotorPIDController.setP(0.5);
    shooterMotorPIDController.setI(0);
    shooterMotorPIDController.setD(0.1);


  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

  }







  public ShooterSubsystemStatus getStatus() {
    return this.shooterSubsystemStatus;
  }

  public void setStatus(ShooterSubsystemStatus shooterSubsystemStatusSet) {
    shooterSubsystemStatus = shooterSubsystemStatusSet;
  }

  public void setRefrence(double speed){
    speedTarget = speed;
    shooterMotorPIDController.setReference(speed, ControlType.kVelocity); 
  }

  public void setMotor(double speed) {
    shooterMotorTop.set(speed);
    shooterMotorBottom.set(speed);
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
