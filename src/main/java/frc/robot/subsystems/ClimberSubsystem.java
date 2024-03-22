package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  // private CANSparkMax leftClimberSPK; 
  private CANSparkMax rightClimberSPK;
  // private VictorSP leftClimberVSP;
  // private VictorSP rightClimberVSP;

  private PWMSparkMax leftClimberPWM; 
  //private PWMSparkMax rightClimberSPK;

  public enum ClimberStatus {
    UNTOUCHED,
    MOVED
  }
  public enum ClimberControlStatus {
    BOTH,
    LEFT_ONLY,
    RIGHT_ONLY,
    NEITHER
  }
  
  public enum LeftClimberStatus {
    LIMIT,
    MOVABLE
  }
  public enum RightClimberStatus {
    LIMIT,
    MOVABLE
  }

  ClimberControlStatus climberControlStatus = ClimberControlStatus.NEITHER;

  public ClimberSubsystem() {
    // leftClimber = new CANSparkMax(Constants.IDConstants.leftClimberMotorID, MotorType.kBrushless);
    rightClimberSPK = new CANSparkMax(27, MotorType.kBrushless);
    leftClimberPWM = new PWMSparkMax(5);
    //rightClimberSPK = new PWMSparkMax(6);

    rightClimberSPK.setInverted(true);
    leftClimberPWM.setInverted(true);

  }

  @Override
  public void periodic() {
    if (Math.abs(rightClimberSPK.get()) > 0.1 && Math.abs(leftClimberPWM.get()) > 0.1 ) {
      climberControlStatus = ClimberControlStatus.BOTH;
    }
    else {
      climberControlStatus = ClimberControlStatus.NEITHER;
    }
  }

  public void setBothMotors(double speed, double speed2) {
    leftClimberPWM.set(MathUtil.applyDeadband(speed, 0.1));
    rightClimberSPK.set((MathUtil.applyDeadband(speed2, 0.1)));
  }

  public void setLeftMotor(double speed) {
    leftClimberPWM.set(speed);
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
