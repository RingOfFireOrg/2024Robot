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
    intakeWheels = new CANSparkMax(Constants.IDConstants.intakeWheelMotorID,MotorType.kBrushless);
    pivotEncoder = new DutyCycleEncoder(0); //Put in Constants

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setMotor(double wheelSpeed) {
    intakeWheels.set(wheelSpeed);
  }










  public CANSparkMax returnIntakeWheels() {
    return intakeWheels;
  }

  public DutyCycleEncoder returnPivotEncoder() {
    return pivotEncoder;
  }



  public void setWheelMotor(double speed) {
    intakeWheels.set(speed);
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
