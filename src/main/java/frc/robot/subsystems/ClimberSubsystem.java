package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  // private CANSparkMax leftClimberSPK; 
  private CANSparkMax rightClimberSPK;
  // private VictorSP leftClimberVSP;
  // private VictorSP rightClimberVSP;

  private PWMSparkMax leftClimberSPK; 
  //private PWMSparkMax rightClimberSPK;

  public enum ClimberStatus {
    UNTOUCHED,
    MOVED
  }
  public enum ClimberControlStatus {
    BOTH,
    LEFT_ONLY,
    RIGHT_ONLY
  }

  public ClimberSubsystem() {
    // leftClimber = new CANSparkMax(Constants.IDConstants.leftClimberMotorID, MotorType.kBrushless);
    rightClimberSPK = new CANSparkMax(27, MotorType.kBrushless);
    leftClimberSPK = new PWMSparkMax(5);
    //rightClimberSPK = new PWMSparkMax(6);

    rightClimberSPK.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }




  public void setMotor(double speed, double speed2) {
    leftClimberSPK.set(speed);
    rightClimberSPK.set(speed2);
  }
   public void setLeftMotor(double speed) {
    leftClimberSPK.set(speed);
  }
   public void setRightMotor(double speed) {
    rightClimberSPK.set(speed);
  }
  // public void setCoast(){
  //   leftClimber.setIdleMode(IdleMode.kCoast);
  //   rightClimber.setIdleMode(IdleMode.kCoast);
  // }
  
  // public void setBreak(){
  //   leftClimber.setIdleMode(IdleMode.kBrake);
  //   rightClimber.setIdleMode(IdleMode.kBrake);
  
  // }
  // public void setMotorStop() {
  //   leftClimber.stopMotor();
  //   rightClimber.stopMotor();

  // }




}
