package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeSubsystemStatus {
    INTAKE_IN,
    INTAKE_OUT,
    IDLE
  }

  private PWMSparkMax intakeWheelsSPK;

  private double intakeWheelsSpeed;
  private IntakeSubsystemStatus intakeSubsystemStatus;

  
  public IntakeSubsystem() {
    intakeWheelsSPK = new PWMSparkMax(1); //TODO: move to constants
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

  /* Returns Intake Wheels Motor */
  public PWMSparkMax getIntakeWheelMotor() {
    return intakeWheelsSPK;
  }

  /* Returns Intake Wheels Status */
  public IntakeSubsystemStatus getStatus() {
    return intakeSubsystemStatus;
  }

  /* Runs the motor with a limited full speed */
  public void setMotor(double wheelSpeed) {
    intakeWheelsSPK.set(wheelSpeed/2.3);
  }

  /* Runs the motor at max speed, but may shred notes/wheels */
  public void setMotorFull(double wheelSpeed) {
    intakeWheelsSPK.set(wheelSpeed);
  }

  /* Stops Intake Wheels */
  public void stopIntakeWheels() {
    intakeWheelsSPK.stopMotor();
  }

  public Command setMotorSpeeds(double speed) {
    return this.run(() -> setMotor(speed));
  }

  public Command setMotorSpeeds(double speed, double divide) {
    return this.run(() -> setMotor(speed/divide));
  }

  public Command stopIntakeWheel() {
    return this.runOnce(() -> stopIntakeWheels());
  }


}
