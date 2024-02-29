package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private VictorSP intakeWheels;

  public enum IntakeSubsystemStatus {
    INTAKE_IN,
    INTAKE_OUT,
    IDLE
  }


  public IntakeSubsystem() {
    //intakeWheels = new CANSparkMax(Constants.IDConstants.intakeWheelMotorID,MotorType.kBrushless);
    intakeWheels = new VictorSP(1);

  }

  @Override
  public void periodic() {
    
  }

  

  public void setMotor(double wheelSpeed) {
    intakeWheels.set(wheelSpeed/2.3);
  }

  public void setWheelMotorStop() {
    intakeWheels.stopMotor();
  }
}
