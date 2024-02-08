package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX shooterMotor; //falcon i would assume?

  public enum ShooterSubsystemStatus {
    READY,
    REVING,
    IDLE
  }
  ShooterSubsystemStatus shooterSubsystemStatus = ShooterSubsystemStatus.IDLE;

  public ShooterSubsystem() {
    shooterMotor = new TalonFX(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }






  public void setMotor(double speed) {
    shooterMotor.set(speed);
  }
  public void setCoast(){
    shooterMotor.setNeutralMode(NeutralModeValue.Coast);
  }
  public void setBreak(){
    shooterMotor.setNeutralMode(NeutralModeValue.Coast);
  }
  public void setMotorStop() {
    shooterMotor.stopMotor();
  }
}
