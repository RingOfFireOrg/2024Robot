// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotIntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakePivot; 

  public PivotIntakeSubsystem() {
    intakePivot = new CANSparkMax(Constants.IDConstants.intakePivotMotorID ,MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }













  

  public CANSparkMax returnIntakePivot() {
    return intakePivot;
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
}