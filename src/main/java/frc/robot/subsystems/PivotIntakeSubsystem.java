// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotIntakeSubsystem extends SubsystemBase {
  
  private CANSparkMax intakePivot;
  // private SparkPIDController intakePivotPidController;
  // private RelativeEncoder intakePivotEncoder;
  
  //private VictorSP intakePivotCim;
  private DutyCycleEncoder pivotEncoder;
  private AnalogInput noteSensor;


  private Rotation2d encoderOffset;
  private final ArmFeedforward pivotFF;
  private final ProfiledPIDController profiledPIDController;
  //private final TrapezoidProfile.Constraints constraints;

  PivotSubsystemStatus pivotSubsystemStatus = PivotSubsystemStatus.INTAKE_UP;
  NoteSesnorStatus noteSesnorStatus = NoteSesnorStatus.NO_NOTE;

  public enum PivotSubsystemStatus {
    INTAKE_UP,
    INTAKE_DOWN,
    INTAKE_MIDAIR
  }

  public enum NoteSesnorStatus {
    NOTE_DECTECTED,
    NO_NOTE,
    VALUE_OUT_OF_BOUNDS
  }

  public PivotIntakeSubsystem() {
    intakePivot = new CANSparkMax(34, MotorType.kBrushless);
    intakePivot.setIdleMode(IdleMode.kCoast);
    // intakePivotPidController = intakePivot.getPIDController();
    // intakePivotEncoder = intakePivot.getEncoder();

    // intakePivotPidController.setP(0);
    // intakePivotPidController.setI(0);
    // intakePivotPidController.setD(0);
    // intakePivotPidController.setFF(0);
    // intakePivotPidController.setOutputRange(-1, 1);
    
    //intakePivotCim = new VictorSP(0); //PWM

    pivotEncoder = new DutyCycleEncoder(0); //DIO

    pivotFF = new ArmFeedforward(0.15, 0.25, 1.0, 0.25);
    profiledPIDController = new ProfiledPIDController
    (4, 0, 0.015, new Constraints(8, 30));
    profiledPIDController.disableContinuousInput();


    noteSensor = new AnalogInput(0);
    noteSensor.setAverageBits(4);

    
    
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pivotEncoderPos = pivotEncoder.getAbsolutePosition();
    SmartDashboard.putNumber("piIntake Position", pivotEncoderPos);
    SmartDashboard.putString("piIntake Status", pivotSubsystemStatus.toString());
    SmartDashboard.putNumber("pi Intake Pivot Get Applied output", intakePivot.getAppliedOutput());
    
    if ((pivotEncoderPos <= 0.2 && pivotEncoderPos >= 0) ) {
      pivotSubsystemStatus = PivotSubsystemStatus.INTAKE_UP;
    }
    else if (pivotEncoderPos <= 0.6 && pivotEncoderPos >= 0.4) {
      pivotSubsystemStatus = PivotSubsystemStatus.INTAKE_DOWN;
    }
    else {
      pivotSubsystemStatus = PivotSubsystemStatus.INTAKE_MIDAIR;
    }




    if (noteSensor.getValue() > 1000 ) {
      noteSesnorStatus = NoteSesnorStatus.NO_NOTE;
    }
    else if (noteSensor.getValue() < 50) {
      noteSesnorStatus = NoteSesnorStatus.NOTE_DECTECTED;
    }
    else {
      noteSesnorStatus = NoteSesnorStatus.VALUE_OUT_OF_BOUNDS;
    }
    SmartDashboard.putString("ns_Note Sensor Status", noteSesnorStatus.toString());
    SmartDashboard.putNumber("ns_Note Sensor Value", noteSensor.getValue());

  }



  public PivotSubsystemStatus getIntakeStatus() {
    return pivotSubsystemStatus;
  }

  public NoteSesnorStatus getNoteSesnorStatus() {
    return noteSesnorStatus;
  }

  public CANSparkMax returnIntakePivotMotor() {
    return intakePivot;
  }
  public DutyCycleEncoder returnIntakePivotEncoder() {
    return pivotEncoder;
  }

  public Command intakeDownPPID() {
    return setIntakePivotPos(new Rotation2d(-1.5708));
  }  
  
  public Command intakeUpPPID() {
    return setIntakePivotPos(new Rotation2d(1.5708));
  }

  public Command setIntakePivotPos(Rotation2d posRad) {
    return this.run(
      () -> {
        intakePivot.setVoltage(calculateVoltage(posRad));
      })
  .finallyDo(() -> intakePivot.setVoltage(0));
  }


  public double calculateVoltage(Rotation2d angle) {
    profiledPIDController.setGoal(angle.getRadians());
    var profileSetpoint = profiledPIDController.getSetpoint();
    double feedForwardVoltage =
        pivotFF.calculate(
            profileSetpoint.position, profileSetpoint.velocity);
    double feedbackVoltage = profiledPIDController.calculate((pivotEncoder.getAbsolutePosition()*10)* (Math.PI/180));

    return feedForwardVoltage + feedbackVoltage;
  }

  /* ------------------------------------- CIM Motor intake ---------------------------------------- */
  public void setPivotMotor(double speed) {
    intakePivot.set(speed);
    //intakePivotCim.set(speed);
  }

  public void intakeDown() {
    if (pivotEncoder.getAbsolutePosition() < 0.7) {
      //intakePivotCim.set(0.7);
      intakePivot.set(0.7);

    } 
  }

  public void intakeDownStatus() {
    while (getIntakeStatus() != PivotSubsystemStatus.INTAKE_DOWN) {
//      intakePivotCim.set(-0.7);
      intakePivot.set(0.7);

    } 
  }

  public void intakeUpStatus() {
    while (getIntakeStatus() != PivotSubsystemStatus.INTAKE_UP) {
      //intakePivotCim.set(0.7);
      intakePivot.set(0.7);

    } 
  }
  /* ----------------------------------------------------------------------------------------- */

  // public void setRefernecePositionControl(double rotations) {
  //   intakePivotPidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  // }

}
