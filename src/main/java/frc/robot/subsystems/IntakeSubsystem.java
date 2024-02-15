// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakePivot; 
  private CANSparkMax intakeWheels;
  private DutyCycleEncoder pivotEncoder; 

  

  public enum IntakeSubsystemStatus {
    READY,
    REVING,
    IDLE
  }

  public enum IntakeSubsystemLocation {
    TRANSFER,
    INTAKEDOWN,
    BETWEEN
  }

  public IntakeSubsystem() {
    intakePivot = new CANSparkMax(Constants.IDConstants.intakePivotMotorID ,MotorType.kBrushless);
    intakeWheels = new CANSparkMax(Constants.IDConstants.intakeWheelMotorID,MotorType.kBrushless);
    pivotEncoder = new DutyCycleEncoder(0); //Put in Constants

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setMotors(double pivotSpeed, double wheelSpeed) {
    intakePivot.set(pivotSpeed);
    intakeWheels.set(wheelSpeed);

  }








  public CANSparkMax returnIntakePivot() {
    return intakePivot;
  }

  public CANSparkMax returnIntakeWheels() {
    return intakeWheels;
  }

  public DutyCycleEncoder returnPivotEncoder() {
    return pivotEncoder;
  }


  public void setPivotMotor(double speed) {
    intakePivot.set(speed);
  }
  public void setPivotCoast(){
    intakePivot.setIdleMode(IdleMode.kCoast);
  }
  public void setPivotBrake(){
    intakePivot.setIdleMode(IdleMode.kBrake);
  }
  public void setPivotMotorStop() {
    intakePivot.stopMotor();
  }
  public void setWheelMotor(double speed) {
    intakeWheels.set(speed);
    SmartDashboard.putNumber(" WHEEEEL SPEEED", speed);
  }
  public void setWheelCoast(){
    intakeWheels.setIdleMode(IdleMode.kCoast);
  }
  public void setWheelBrake(){
    intakeWheels.setIdleMode(IdleMode.kBrake);
  }
  public void setWheelMotorStop() {
    intakeWheels.stopMotor();
  }
}
