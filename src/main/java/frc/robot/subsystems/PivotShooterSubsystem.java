package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotShooterSubsystem extends SubsystemBase {

  private TalonFX pivotMotor;
 // private DutyCycleEncoder pivotEncoder; 

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

  public PivotShooterSubsystem() {
    pivotMotor = new TalonFX(Constants.IDConstants.shooterPivotMotorID);
   // pivotEncoder = new DutyCycleEncoder(0);
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
