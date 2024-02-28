// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private VictorSP intakeWheels;

  

  public enum IntakeSubsystemStatus {
    READY,
    REVING,
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
    intakeWheels.set(wheelSpeed);
  }

  // public PWMSparkMax returnIntakeWheels() {
  //   return intakeWheels;
  // }

  public void setWheelMotor(double speed) {
    intakeWheels.set(speed/2);
  }
  /*public void setWheelCoast(){
    intakeWheels.setIdleMode(IdleMode.kCoast);
  }
  public void setWheelBrake(){
    intakeWheels.setIdleMode(IdleMode.kBrake);
  }*/
  public void setWheelMotorStop() {
    intakeWheels.stopMotor();
  }
}
