// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class PivotIntakeSubsystem extends SubsystemBase {
  
  private CANSparkMax intakePivot;
  // private SparkPIDController intakePivotPidController;
  private RelativeEncoder intakePivotEncoder;
  PIDController intakePivotPIDController;
  PIDController intakePivotPIDController_ABS;
  PivotSubsystemStatus pivotSubsystemStatus = PivotSubsystemStatus.INTAKE_UP;
  NoteSesnorStatus noteSesnorStatus = NoteSesnorStatus.NO_NOTE;
  private DutyCycleEncoder pivotEncoder;
  private AnalogInput noteSensor;
  private DigitalInput noteSensorDIO;
  private ProfiledPIDController intakePivotPPIDController;
  SimpleMotorFeedforward intakFeedforward;
  Timer timer;

  //private Rotation2d encOffset;
  // private final ArmFeedforward pivotFF;
  //private final ProfiledPIDController profiledPIDController;


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
    intakePivot.burnFlash();
    // intakePivotPidController = intakePivot.getPIDController();

    // intakePivotPidController.setP(0);
    // intakePivotPidController.setI(0);
    // intakePivotPidController.setD(0);
    // intakePivotPidController.setFF(0);
    // intakePivotPidController.setOutputRange(-1, 1);
    


    // pivotFF = new ArmFeedforward(0.15, 0.25, 1.0, 0.25);
    // profiledPIDController = new ProfiledPIDController
    // (4, 0, 0.015, new Constraints(8, 30));
    // profiledPIDController.disableContinuousInput();

    /* Creating Instance of a PID Controller using the Built in Encoder */
    intakePivotEncoder = intakePivot.getEncoder();
    intakePivotPIDController = new PIDController(0.009, 0, 0);
    intakePivotPIDController.setTolerance(0.1,0.01);

    /* Creating Instance of a PID Controller using the Absolute Encoder */
    pivotEncoder = new DutyCycleEncoder(0); //TODO: DIO, move to constants
    intakePivotPIDController_ABS = new PIDController(1, 0, 0.2);
    intakePivotPIDController_ABS.setTolerance(0.05,0.01);

    intakePivotPPIDController = new ProfiledPIDController
    (3.5, 0, 0.2,
    new TrapezoidProfile.Constraints(20,20));
    intakePivotPPIDController.setTolerance(0.009, 0.07);

    intakFeedforward = new SimpleMotorFeedforward(0.175, 0.2, 1.0);


    noteSensor = new AnalogInput(0); //TODO: move to constants
    noteSensor.setAverageBits(4);

    noteSensorDIO = new DigitalInput(4);

    
    

  }

  @Override
  public void periodic() {

    /* Sending Intake Position Data */
    double pivotEncoderPos = pivotEncoder.getAbsolutePosition();
    SmartDashboard.putNumber("piIntake Position", pivotEncoderPos);
    SmartDashboard.putString("piIntake Status", pivotSubsystemStatus.toString());
    SmartDashboard.putNumber("pi_ Intake Pivot Get Applied output", intakePivot.getAppliedOutput());
    SmartDashboard.putNumber("pi Intake Motor Controller encoder", intakePivotEncoder.getPosition());
    if ((pivotEncoderPos <= 0.4 && pivotEncoderPos >= 0) ) {
      pivotSubsystemStatus = PivotSubsystemStatus.INTAKE_UP;
    }
    else if (pivotEncoderPos <= 1 && pivotEncoderPos >= 0.82) {
      pivotSubsystemStatus = PivotSubsystemStatus.INTAKE_DOWN;
    }
    else {
      pivotSubsystemStatus = PivotSubsystemStatus.INTAKE_MIDAIR;
    }



    /* Sending Note Sensor Data */
    SmartDashboard.putString("ns_Note Sensor Status", noteSesnorStatus.toString());
    SmartDashboard.putNumber("ns_Note Sensor Value", noteSensor.getValue());

    if (noteSensor.getValue() > 1000 || noteSensorDIO.get() == false) {
      noteSesnorStatus = NoteSesnorStatus.NO_NOTE;
    }
    else if (noteSensor.getValue() < 50 || noteSensorDIO.get() == true) {
      noteSesnorStatus = NoteSesnorStatus.NOTE_DECTECTED;
    }
    else {
      noteSesnorStatus = NoteSesnorStatus.VALUE_OUT_OF_BOUNDS;
    }


    SmartDashboard.putNumber("AS1_pi_PPID TO poisition", intakePivotPPIDController.calculate(
      pivotEncoder.getAbsolutePosition(), intakePivotPPIDController.getGoal())+ intakFeedforward.calculate(intakePivotPPIDController.getSetpoint().velocity));
    SmartDashboard.putNumber("AS1_pi_cSet", intakePivot.get());

  }


  /* -----------------------------------------------  */
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
  /* ------------------------------------------------ */




 /*-------------------------------------------------- */

  /* Method Using Built in motor encoder */
  public void toPosition(double targetPosition) {     
    SmartDashboard.putNumber("pi_to_position", intakePivotPIDController.calculate(intakePivotEncoder.getPosition(), targetPosition));
    intakePivot.set(
      intakePivotPIDController.calculate(
        intakePivotEncoder.getPosition(), targetPosition));
    // -5 to 34 (Assuming reset is up)
  }

  public Command intakeUpPID() {
    return this
    .run(() -> {toPosition(-0.5);}) //TODO: move to constants
    .until(() -> (pivotSubsystemStatus == PivotSubsystemStatus.INTAKE_UP));
  }

  public Command intakeDownPID() {
    return this
    .run(() -> {toPosition(36);})//TODO: move to constants
    .until(() -> (pivotSubsystemStatus == PivotSubsystemStatus.INTAKE_DOWN));
  }


  /* Method Using External Encoder */
  public void toPositionABS(double targetPosition) {  
    SmartDashboard.putNumber("pi_to_positionABS", intakePivotPIDController.calculate( intakePivotEncoder.getPosition(), targetPosition));    
    intakePivot.set(
      intakePivotPIDController_ABS.calculate(
        pivotEncoder.get(), targetPosition));
  }

  public Command intakeUpPID_ABS() {
    return this
    .run(() -> {toPositionABS(0.4);})//TODO: move to constants
    .until(() -> (pivotSubsystemStatus == PivotSubsystemStatus.INTAKE_UP))
    .finallyDo(() -> stopMotor());
  }

  public Command intakeDownPID_ABS() {
    return this
    .run(() -> {toPositionABS(0.85);})//TODO: move to constants
    .until(() -> (pivotSubsystemStatus == PivotSubsystemStatus.INTAKE_DOWN))
    .finallyDo(() -> stopMotor());
  }


/*-----------------------------------------------------------*/



/* ------------------ Profiled PID Testing --------------- */


  public void toPositionPPID(double goal) {
    // SmartDashboard.putNumber("AS1_pi_PPID TO poisition", intakePivotPPIDController.calculate(
    //   pivotEncoder.getAbsolutePosition(), goal)+ intakFeedforward.calculate(intakePivotPPIDController.getSetpoint().velocity));
    // SmartDashboard.putNumber("AS1_pi_cSet", intakePivot.get());
      
    intakePivot.set(
      intakePivotPPIDController.calculate(
        pivotEncoder.getAbsolutePosition(), goal)+ intakFeedforward.calculate(intakePivotPPIDController.getSetpoint().velocity));
  }

  public Command intakeUpPPID() {
    return this
    .run(() -> toPositionPPID(0.4))
   // .until(() -> intakePivotPIDController.atSetpoint())

    .until(() -> pivotSubsystemStatus == PivotSubsystemStatus.INTAKE_UP)
    .finallyDo(() -> stopMotor())
    ;
  }

  public Command intakeDownPPID() {
    return this
    .run(() -> toPositionPPID(0.85))
   // .until(() -> intakePivotPIDController.atSetpoint())

    .until(() -> pivotSubsystemStatus == PivotSubsystemStatus.INTAKE_DOWN)
    .finallyDo(() -> stopMotor())
    ;
  }

  // public void voltagePPID() {
  //   intakePivot.setVoltage(
  //     intakePivotPPIDController.calculate(pivotEncoder.getDistance())
  //         + m_feedforward.calculate(m_controller.getSetpoint().velocity));
  // }
  // public void voltagePPID(double goal) {
  //   intakePivot.setVoltage(
  //     intakePivotPPIDController
  //     .calculate(pivotEncoder.getAbsolutePosition(), goal)
  //     + //feedforward here(?)
  //   );
  // }
  // public Command intakeDownPPID() {
  //   return setIntakePivotPos(new Rotation2d(-1.5708));
  // }  
  
  // public Command intakeUpPPID() {
  //   return setIntakePivotPos(new Rotation2d(1.5708));
  // }

  // public Command setIntakePivotPos(Rotation2d posRad) {
  //   return this.run(
  //     () -> {
  //       intakePivot.setVoltage(calculateVoltage(posRad));
  //     })
  // .finallyDo(() -> intakePivot.setVoltage(0));
  // }


  // public double calculateVoltage(Rotation2d angle) {
  //   profiledPIDController.setGoal(angle.getRadians());
  //   var profileSetpoint = profiledPIDController.getSetpoint();
  //   double feedForwardVoltage =
  //       pivotFF.calculate(
  //           profileSetpoint.position, profileSetpoint.velocity);
  //   double feedbackVoltage = profiledPIDController.calculate((pivotEncoder.getAbsolutePosition()*10)* (Math.PI/180));

  //   return feedForwardVoltage + feedbackVoltage;
  // }





  /* ------------------------------------- Open Loop Cycle Movement ---------------------------------------- */

  public void setPivotMotor(double speed) {
    intakePivot.set(speed);
  }

  public void intakeDownStatus() {
    while (getIntakeStatus() != PivotSubsystemStatus.INTAKE_DOWN) {
      intakePivot.set(0.7);
    } 
  }

  public void intakeUpStatus() {
    while (getIntakeStatus() != PivotSubsystemStatus.INTAKE_UP) {
      intakePivot.set(0.7);
    } 
  }

  public void stopMotor() {
    intakePivot.stopMotor();
  }
  
  /* ----------------------------------------------------------------------------------------------------- */
  
  public Command noteCheckCMD() {
    return new WaitUntilCommand(() -> noteSesnorStatus == NoteSesnorStatus.NOTE_DECTECTED);
  }

  public Command noteCheckTimerCMD() {
    return new WaitUntilCommand(() -> noteTimer());
  }

  public boolean noteTimer() {
    if(noteSesnorStatus == NoteSesnorStatus.NOTE_DECTECTED) {
      timer.reset();
      while (timer.hasElapsed(0.20) == false) {
        if (noteSesnorStatus != NoteSesnorStatus.NOTE_DECTECTED) {
          return false;
        }
      }
    }
    return true;
  }



}
