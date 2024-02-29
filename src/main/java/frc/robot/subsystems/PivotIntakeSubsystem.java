// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotIntakeSubsystem extends SubsystemBase {
  
  //private CANSparkMax intakePivot; 
  private VictorSP intakePivotCim;
  private DutyCycleEncoder pivotEncoder;

  PivotSubsystemStatus pivotSubsystemStatus = PivotSubsystemStatus.INTAKE_UP;

  public enum PivotSubsystemStatus {
    INTAKE_UP,
    INTAKE_DOWN,
    INTAKE_MIDAIR
  }

  public PivotIntakeSubsystem() {
    //intakePivot = new CANSparkMax(Constants.IDConstants.intakePivotMotorID ,MotorType.kBrushless);
    intakePivotCim = new VictorSP(0); //PWM
    pivotEncoder = new DutyCycleEncoder(1); //DIO
    pivotEncoder.setPositionOffset(0.8);
    
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pivotEncoderPos = pivotEncoder.getAbsolutePosition();
    SmartDashboard.putNumber("Intake Position", pivotEncoderPos);
    SmartDashboard.putString("Intake Status", pivotSubsystemStatus.toString());
    if (pivotEncoderPos <= 0.3 && pivotEncoderPos >= 0) {
      pivotSubsystemStatus = PivotSubsystemStatus.INTAKE_UP;
    }
    else if (pivotEncoderPos <= 1 && pivotEncoderPos >= 0.7) {
      pivotSubsystemStatus = PivotSubsystemStatus.INTAKE_DOWN;
    }
    else {
      pivotSubsystemStatus = PivotSubsystemStatus.INTAKE_MIDAIR;
    }
  }

  public PivotSubsystemStatus getIntakeStatus() {
    return pivotSubsystemStatus;
  }

  // public CANSparkMax returnIntakePivotMotor() {
  //   return intakePivot;
  // }
  public DutyCycleEncoder returnIntakePivotEncoder() {
    return pivotEncoder;
  }
  
  public void setPivotMotor(double speed) {
    //intakePivot.set(speed);
    intakePivotCim.set(speed);
  }

  public void intakeDown() {
    if (pivotEncoder.getAbsolutePosition() < 0.7) {
      intakePivotCim.set(0.7);
    } 
  }
  // public void setPivotCoast(){
  //   intakePivot.setIdleMode(IdleMode.kCoast);
  // }
  // public void setPivotBrake(){
  //   intakePivot.setIdleMode(IdleMode.kBrake);
  // }
  // public void setPivotMotorStop() {
  //   intakePivot.stopMotor();
  // }
}
