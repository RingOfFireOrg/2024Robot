package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeSubsystemStatus {
    INTAKE_IN,
    INTAKE_OUT,
    IDLE
  }

  //private VictorSP intakeWheels;
  private PWMSparkMax intakeWheelsSPK;
  private double intakeWheelsSpeed;

  IntakeSubsystemStatus intakeSubsystemStatus;
  public IntakeSubsystem() {
    //intakeWheels = new CANSparkMax(Constants.IDConstants.intakeWheelMotorID,MotorType.kBrushless);
    //intakeWheels = new VictorSP(1);
    intakeWheelsSPK = new PWMSparkMax(1);

  }

  @Override
  public void periodic() {

    intakeWheelsSpeed = intakeWheelsSPK.get();

    if (intakeWheelsSpeed > 0.15) {
      intakeSubsystemStatus = IntakeSubsystemStatus.INTAKE_IN;
    }
    else if (intakeWheelsSpeed < -0.15) { 
      intakeSubsystemStatus = IntakeSubsystemStatus.INTAKE_OUT;
    }
    else { 
      intakeSubsystemStatus = IntakeSubsystemStatus.IDLE;
    }

    SmartDashboard.putString("iw_Intake Wheels Status", intakeSubsystemStatus.toString());
    SmartDashboard.putNumber("iw_Intake Wheels Speed", intakeWheelsSpeed);

  }


  public PWMSparkMax getIntakeWheelMotor() {
    return intakeWheelsSPK;
  }

  public IntakeSubsystemStatus getStatus() {
    return intakeSubsystemStatus;
  }

  /* Runs the motor with a limited full speed */
  public void setMotor(double wheelSpeed) {
    //intakeWheels.set(wheelSpeed/2.3);
    intakeWheelsSPK.set(wheelSpeed/2.3);
  }

  /* Runs the motor at max speed, but may shred notes/wheels */
  public void setMotorFull(double wheelSpeed) {
    intakeWheelsSPK.set(wheelSpeed);
  }

  public void setWheelMotorStop() {
    //intakeWheels.stopMotor();
    intakeWheelsSPK.stopMotor();
  }


}
