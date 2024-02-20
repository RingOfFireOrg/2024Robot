// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax leftClimber; 
  private CANSparkMax rightClimber;

  public enum ClimberStatus {
    UNTOUCHED,
    MOVED
  }
  public enum ClimberControlStatus {
    BOTH,
    LEFT_ONLY,
    RIGHT_ONLY
  }

  public ClimberSubsystem() {
    leftClimber = new CANSparkMax(Constants.IDConstants.leftClimberMotorID, MotorType.kBrushless);
    rightClimber = new CANSparkMax(Constants.IDConstants.rightClimberMotorID, MotorType.kBrushless);

    rightClimber.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }




  public void setMotor(double speed) {
    leftClimber.set(speed);
    rightClimber.set(speed);
  }
   public void setLeftMotor(double speed) {
    leftClimber.set(speed);
    // rightClimber.set(speed);
  }
   public void setRightMotor(double speed) {
    // leftClimber.set(speed);
    rightClimber.set(speed);
  }
  public void setCoast(){
    leftClimber.setIdleMode(IdleMode.kCoast);
    rightClimber.setIdleMode(IdleMode.kCoast);
  }
  
  public void setBreak(){
    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);
  
  }
  public void setMotorStop() {
    leftClimber.stopMotor();
    rightClimber.stopMotor();

  }




}
