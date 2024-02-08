package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

  private TalonFX pivotMotor;
  private CANcoder pivotCANcoder; //we need to look into if we have spares absolute encoders, I think relative encoders will be problamtic

  public enum PivotAngleStatus {
    HOME,
    ANGLED,
    AMP
  }
  public enum PivotSubsystemStatus {
    REST,
    ANGLING,
    CONTROLED
  }
  PivotAngleStatus pivotAngleStatus;
  PivotSubsystemStatus pivotSubsystemStatus;

  public PivotSubsystem() {
    pivotMotor = new TalonFX(0);
    pivotCANcoder = new CANcoder(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }











  public void setMotor(double speed) {
    pivotMotor.set(speed);
  }
  public void setCoast(){
    pivotMotor.setNeutralMode(NeutralModeValue.Coast);
  }
  public void setBreak(){
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  public void setMotorStop() {
    pivotMotor.stopMotor();
  }

}
