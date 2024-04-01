package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LEDAutoStatus;
import frc.robot.commands.LEDTeleOpStatus;
import frc.robot.commands.SwerveNoteTrack;
import frc.robot.commands.AutoCommands.IntakeDown;
import frc.robot.commands.AutoCommands.IntakeUp;
import frc.robot.commands.AutoCommands.TransferCMD;
import frc.robot.commands.TeleopCommands.ClimberTeleop;
import frc.robot.commands.TeleopCommands.IntakePivotTeleop;
import frc.robot.commands.TeleopCommands.IntakeTeleop;
import frc.robot.commands.TeleopCommands.KrakenShooterTeleop;
import frc.robot.commands.TeleopCommands.SwerveJoystickCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KrakenShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.KrakenShooterSubsystem.KrakenShooterSubsystemStatus;
import frc.robot.subsystems.PivotIntakeSubsystem.NoteSesnorStatus;
import frc.robot.subsystems.Vision.LimelightHelpers;


public class RobotContainer {
  public LEDSubsystem ledSubsystem = new LEDSubsystem();
  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public PivotIntakeSubsystem pivotIntakeSubsystem = new PivotIntakeSubsystem();
  public KrakenShooterSubsystem krakenShooterSubsystem = new KrakenShooterSubsystem();
  public ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  //public LimeLight limeLightSubsystem = new LimeLight();
  public PhotonVisionSubsystem photonVisionSubsystem = new PhotonVisionSubsystem();
  //TODO Comment out

  XboxController driverController = new XboxController(OIConstants.driverControllerPort);
  CommandXboxController operatorController = new CommandXboxController(OIConstants.operatorControllerPort);
  XboxController climberController = new XboxController(OIConstants.climberControllerPort); 

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  
  ShuffleboardTab codeTestTab = Shuffleboard.getTab("Code Testing");

  GenericEntry amp_topShooter = codeTestTab
    .add("(A)Top Shooter", 900)
    // .withWidget(BuiltInWidgets.kNumberSlider)
    // .withProperties(Map.of("min", -1000, "max", 1000))
    .getEntry();

  GenericEntry amp_bottomShooter = codeTestTab
    .add("(A)Bottom Shooter", 500)
    // .withWidget(BuiltInWidgets.kNumberSlider)
    // .withProperties(Map.of("min", -1000, "max", 1000))
    .getEntry();


  public RobotContainer() {
    namedCommands();
    populateAutoChooser();
    defaultCommands();
    configureButtonBindings();

  }

  /* Sets default commands for each subsystem */
  private void defaultCommands() {

    ledSubsystem.setDefaultCommand(new LEDTeleOpStatus(
      ledSubsystem, 
      () -> krakenShooterSubsystem.getStatus(),              //Supplier for Shooter Status
      () -> pivotIntakeSubsystem.getIntakeStatus(),    //Supplier for Intake Pivot Status
      () -> intakeSubsystem.getStatus(),               //Supplier for Intake Wheels
      () -> pivotIntakeSubsystem.getNoteSesnorStatus() //Supplier for InfaredSensor
    ));

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
      swerveSubsystem,

      // Left Joystick Field Oriented
      () -> -driverController.getRawAxis(OIConstants.leftStickY),
      () -> driverController.getRawAxis(OIConstants.leftStickX),

      //Right Joystick For Robot Centic
      () -> -driverController.getRawAxis(OIConstants.rightStickY),
      () -> driverController.getRawAxis(OIConstants.rightStickX),

      // Triggers for turning
      () -> driverController.getRawAxis(OIConstants.rightTrigger),
      () -> driverController.getRawAxis(OIConstants.leftTrigger),

      //Varied Assortment of Buttons to click
      () -> !driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),

      // Speed Buttons
      () -> driverController.getRawButton(OIConstants.aButton),
      () -> driverController.getRawButton(OIConstants.bButton),
      () -> driverController.getRawButton(OIConstants.xButton),
      () -> driverController.getRawButton(OIConstants.yButton),


      () -> driverController.getRawButton(OIConstants.kAlignWithTargetButton),
      () -> driverController.getRawButton(OIConstants.kResetDirectionButton)
    ));

    // shooterSubsystem.setDefaultCommand(new ShooterTeleop(
    //   shooterSubsystem, 
    //   () -> (- operatorController.getRightTriggerAxis()) 
    // ));

    krakenShooterSubsystem.setDefaultCommand(
      new KrakenShooterTeleop(krakenShooterSubsystem, () -> (operatorController.getLeftTriggerAxis() -  operatorController.getRightTriggerAxis()))
    );

    pivotIntakeSubsystem.setDefaultCommand(new IntakePivotTeleop(
      pivotIntakeSubsystem, 
      () -> operatorController.getLeftY())
    );

    intakeSubsystem.setDefaultCommand(new IntakeTeleop(
      intakeSubsystem, 
      () -> operatorController.getRightY()
    ));


    climberSubsystem.setDefaultCommand(new ClimberTeleop(
      climberSubsystem, 
      () -> climberController.getLeftY(),
      () -> climberController.getRightY()
    ));
  }

  /* Registers Named Commands used in Paths */
  private void namedCommands() {

    /* Shooter Commands */
    NamedCommands.registerCommand("ShootOn", new InstantCommand( () -> krakenShooterSubsystem.setMotor(0.6), krakenShooterSubsystem));
    NamedCommands.registerCommand("ShootIdle", new InstantCommand( () -> krakenShooterSubsystem.setMotor(0.5), krakenShooterSubsystem));
    NamedCommands.registerCommand("ShootOff", new InstantCommand( () -> krakenShooterSubsystem.setMotor(0), krakenShooterSubsystem));

    NamedCommands.registerCommand("ShootOnRPM", new InstantCommand( () -> krakenShooterSubsystem.setRPM(-3100), krakenShooterSubsystem));
    NamedCommands.registerCommand("ShootIdleRPM", new InstantCommand( () -> krakenShooterSubsystem.setRPM(-1000), krakenShooterSubsystem));
    NamedCommands.registerCommand("ShootOffRPM", new InstantCommand( () -> krakenShooterSubsystem.setRPM(0), krakenShooterSubsystem));

    /* Intake Wheel Commands */
    NamedCommands.registerCommand("IntakeIn", new InstantCommand( () -> intakeSubsystem.setMotorFull(0.8), intakeSubsystem));
    NamedCommands.registerCommand("IntakeInSlowly", new InstantCommand( () -> intakeSubsystem.setMotorFull(0.5), intakeSubsystem));

    NamedCommands.registerCommand("IntakeOut", new InstantCommand( () -> intakeSubsystem.setMotor(-1), intakeSubsystem));
    NamedCommands.registerCommand("IntakeOutSlowly", new InstantCommand( () -> intakeSubsystem.setMotor(-0.6), intakeSubsystem));

    NamedCommands.registerCommand("IntakeOff", new InstantCommand( () -> intakeSubsystem.setMotor(0), intakeSubsystem));

    /* Only use for Centerline auto to try and shoot the last shot extremely fast */
    NamedCommands.registerCommand("IntakeOutBlitz", new InstantCommand( () -> intakeSubsystem.setMotorFull(0.9), intakeSubsystem));

    /* Intake Pivot Commands */
    // NamedCommands.registerCommand("IntakeUp", new IntakeUp(pivotIntakeSubsystem, 0.5));
    // NamedCommands.registerCommand("IntakeDown", new IntakeDown(pivotIntakeSubsystem, 0.5));

    NamedCommands.registerCommand("IntakeUp", pivotIntakeSubsystem.intakeUpPPID());
    NamedCommands.registerCommand("IntakeDown", pivotIntakeSubsystem.intakeDownPPID());

    /* LED Command */
    NamedCommands.registerCommand("Auto LEDS",new LEDAutoStatus(ledSubsystem, () -> krakenShooterSubsystem.getStatus(), () -> pivotIntakeSubsystem.getIntakeStatus(), () -> intakeSubsystem.getStatus()));

    /*  These commands are so we dont get a bunch of errors yelling at us, they are from old deleted paths that are stuck on the rio */
    NamedCommands.registerCommand("IntakeInDeadline", new InstantCommand());
    NamedCommands.registerCommand("RevShooter", new InstantCommand());
    NamedCommands.registerCommand("TransferRingToShooter", new InstantCommand());
    //NamedCommands.registerCommand("Shoot", new ShootCMD(krakenShooterSubsystem));
    NamedCommands.registerCommand("Transfer", new TransferCMD(intakeSubsystem));


    NamedCommands.registerCommand("Note Check", new InstantCommand());
    //NamedCommands.registerCommand("Note Check", pivotIntakeSubsystem.noteCheckCMD());
    NamedCommands.registerCommand("Note Check Timer", pivotIntakeSubsystem.noteCheckTimerCMD());

  }




  /* Populates the Sendable Chooser to pick autonomous in SmartDashboard */
  private void populateAutoChooser() {

    /*    Planned Auto List
      DONE - Anywhere 1p (Preload)

      2) DONE - Middle 2p (Preload, MiddleRing)
      3) DONE - Middle 3p Up (Preload, MiddleRing, TopRing)
      4) DONE - Middle 3p Down (Preload, MiddleRing, BottomRing)
      5) DONE - Middle 4p (Preload, MiddleRing, TopRing, BottomRing)
      6) DONE - Middle 4p Up (Preload, MiddleRing, TopRing, Centerline3)
      7) DONE -Middle 4p Down (Preload, MiddleRing, BottomRing, Centerline3)

      8) Middle 5p (Preload, MiddleRing, TopRing, BottomRing, Centerline2)

      Top 2p (Preload, TopRing)
      Top 3p (Preload, TopRing, Centerline1)

      Bottom 2p (Preload, BottomRing)
      Bottom 2p C4 (Preload, Centerline4)
      Bottom 3pC4 (Preload, BottomRing, Centerline4)
      Bottom 3p C4C5 (Preload, Centerline4, Centerline5)
      Bottom 4p (Preload, BottomRing, Centerline4, Centerline5) - WILL RUN OUT OF TIME, only use so we can pick up and start with an extra note 
     */

    // make sure to add a wait command at the end of every single auto

    autoChooser.setDefaultOption("Shoot No Movement", new PathPlannerAuto("IntakeTransferTest"));
    autoChooser.addOption("Nothing", new InstantCommand());
    // autoChooser.addOption("Taxi", new PathPlannerAuto("Taxi"));

    /* Pembroke Tuned Autos */
    // autoChooser.addOption("4 Piece", new PathPlannerAuto("MiddleFull"));
    // autoChooser.addOption("3 Middle Left", new PathPlannerAuto("Middle3pLeft"));
    // autoChooser.addOption("2 Middle", new PathPlannerAuto("Middle1"));
    // autoChooser.addOption("2 Left", new PathPlannerAuto("Left2p"));
    // autoChooser.addOption("2 Right", new PathPlannerAuto("Right2p"));
    // autoChooser.addOption("2 left centerstage", new PathPlannerAuto("Left2p to center"));

    /* Wake County Autos */
    autoChooser.addOption("2) Middle 2p", new PathPlannerAuto("2) Middle 2p P-MR"));
    autoChooser.addOption("3) Middle 3p Up", new PathPlannerAuto("3) Middle 3p Up"));
    autoChooser.addOption("4) Middle 3p DOWN", new PathPlannerAuto("4) Middle 3p DOWN"));
    autoChooser.addOption("5) Midlle 4p", new PathPlannerAuto("5) Middle 4p Middle"));
    autoChooser.addOption("6) Middle 4p Up C3", new PathPlannerAuto("6) Middle 4p Up C3"));
    autoChooser.addOption("7) Middle 4p Down C3", new PathPlannerAuto("7) Middle 4p Down C3"));

    autoChooser.addOption("9) Top 2p", new PathPlannerAuto("9) Top 2p"));
    autoChooser.addOption("10) Top 3p", new PathPlannerAuto("10) Top 3p"));

    autoChooser.addOption("11) 2p BR", new PathPlannerAuto("11) 2p BR"));
    autoChooser.addOption("12) 2p C4", new PathPlannerAuto("12) 2p C4"));
    autoChooser.addOption("13) 3p Br+C4", new PathPlannerAuto("13) 3p Br+C4"));
    autoChooser.addOption("14) 3p C4+C5", new PathPlannerAuto("14) 3p C4+C5"));
    autoChooser.addOption("15) 4p Br+C4+C5", new PathPlannerAuto("15) 4p Br+C4+C5"));
    autoChooser.addOption("Intake Test", new PathPlannerAuto("IntakeMoveTest"));
    autoChooser.addOption("STATE_TEST", new PathPlannerAuto("Middle Path est"));
    autoChooser.addOption("STATE_TEST2", new PathPlannerAuto("STATE_2p MiddleRing Race"));

    
    /* Path tuning */
    autoChooser.addOption("Taxi", new PathPlannerAuto("1meter"));
    //autoChooser.addOption("Rotate 180( & move 2 meters)", new PathPlannerAuto("Rotate 180"));

    

    SmartDashboard.putData(autoChooser);
  }



  /* Create button Bindings*/
  private void configureButtonBindings() {

    // use "controller.getHID()" to use it as a standard XboxContoller instead of CommandXboxController

    /* Auto Pickup */
    new JoystickButton(driverController, Constants.OIConstants.leftBumper)
      .whileTrue(new SwerveNoteTrack(swerveSubsystem));
    
    /* Runs the Shooter at a speed desirable for Amping */
    new JoystickButton(operatorController.getHID(), Constants.OIConstants.xButton)
      .whileTrue(krakenShooterSubsystem.rpmCMD(
        () -> amp_topShooter.getDouble(500),
        () -> amp_bottomShooter.getDouble(500)
      )
    );

    /* Speed Testing */
    new JoystickButton(operatorController.getHID(), Constants.OIConstants.aButton)
      .whileTrue(krakenShooterSubsystem.changeSpeed()
    );

    /* Spins Intake in and intake wheels to intake from source faster */
    new JoystickButton(operatorController.getHID(), Constants.OIConstants.leftBumper)
      .whileTrue(intakeSubsystem.setMotorSpeeds(0.3)
        .alongWith(krakenShooterSubsystem.intakeSpeed())
        //.onlyWhile(() -> pivotIntakeSubsystem.getNoteSesnorStatus() != NoteSesnorStatus.NOTE_DECTECTED)
      )
      .onFalse(intakeSubsystem.stopIntakeWheel()
        .alongWith(krakenShooterSubsystem.stopMotorsCMD())
    );


    //TODO: comment out
    new JoystickButton(operatorController.getHID(), Constants.OIConstants.rightBumper)
      .whileTrue(krakenShooterSubsystem.distanceShot(
        photonVisionSubsystem.getMeters(7, 7)
      ))
    ;




    // new JoystickButton(operatorController.getHID(), Constants.OIConstants.rightBumper)
    //   .whileTrue(krakenShooterSubsystem.rpmCMD(-3200)
    //   .andThen(intakeSubsystem.setMotorSpeeds(-0.6).withTimeout(1))
    //   //.unless(() -> krakenShooterSubsystem.getStatus() != KrakenShooterSubsystemStatus.READY)
    //   .andThen(intakeSubsystem.stopIntakeWheel())
    //   .alongWith(krakenShooterSubsystem.stopMotorsCMD())
    // );

    /* Open Loop Control Intake */
    new POVButton(operatorController.getHID(), Constants.OIConstants.dPadUp)
      .onTrue(new IntakeUp(pivotIntakeSubsystem, 0.4)
    );
    new POVButton(operatorController.getHID(), Constants.OIConstants.dPadDown)
      .onTrue(new IntakeDown(pivotIntakeSubsystem, 0.4)
    );
    
    /* PID Closed Loop Control (Motor Encoder) */
    // new POVButton(operatorController.getHID(), Constants.OIConstants.dPadLeft)
    //   .whileTrue(pivotIntakeSubsystem.intakeUpPID()
    // );
    // new POVButton(operatorController.getHID(), Constants.OIConstants.dPadRight)
    //   .whileTrue(pivotIntakeSubsystem.intakeDownPID()
    // );


    // /* PID Closed Loop Control (Absolute Encoder) */
    // new POVButton(operatorController.getHID(), Constants.OIConstants.dPadLeft)
    //   //.whileTrue(pivotIntakeSubsystem.intakeUpPID()
    //   .onTrue(pivotIntakeSubsystem.intakeUpPPID()
    // );
    // new POVButton(operatorController.getHID(), Constants.OIConstants.dPadRight)
    //   //.whileTrue(pivotIntakeSubsystem.intakeDownPID()
    //   .onTrue(pivotIntakeSubsystem.intakeDownPPID()

    // );
    
    // /* Auto Deploy the Intake */      //Combine both into one if the infared sensor does not work    
    // operatorController.axisGreaterThan(Constants.OIConstants.leftTrigger, 0.3) 
    // //.and(() ->(pivotIntakeSubsystem.getNoteSesnorStatus() != NoteSesnorStatus.NOTE_DECTECTED))
    // .onTrue(new IntakeDown(pivotIntakeSubsystem, 0.5)
    //   .alongWith(new IntakeTeleop(intakeSubsystem,() -> 1.0))); //Change to Instant Command?
    
    // /* Intake Auto Stow upon Release */
    // operatorController.axisGreaterThan(Constants.OIConstants.leftTrigger, 0.3)
    //   .onFalse(new IntakeUp(pivotIntakeSubsystem, 0.5)
    //     .alongWith(new IntakeTeleop(intakeSubsystem,() -> 0.0)));


    /* Intake Auto Stow using Infared Sensor */
    // operatorController.axisGreaterThan(Constants.OIConstants.leftTrigger, 0.3).and(() -> (pivotIntakeSubsystem.getNoteSesnorStatus() != NoteSesnorStatus.NOTE_DECTECTED))
    //   .onFalse(new IntakeUp(pivotIntakeSubsystem, 0.5)
    //     .alongWith(new IntakeTeleop(intakeSubsystem,() -> 0.0)));
    // ;

    // operatorController.axisGreaterThan(0, 0)
    //     .onTrue(new IntakeDown(pivotIntakeSubsystem, 0.5)
    //       .alongWith(new IntakeTeleop(intakeSubsystem,() -> 1.0)
    //     )
    // .onlyIf(() -> pivotIntakeSubsystem.getNoteSesnorStatus() != NoteSesnorStatus.NOTE_DECTECTED).end(false));
    // )
    // ; 


    

    //new JoystickButton(driverController, Constants.OIConstants.rightBumper).whileTrue(new TurnToClimb(swerveSubsystem));
    //new JoystickButton(driverController, Constants.OIConstants.leftBumper).whileTrue(new OTFPathGen(swerveSubsystem));
  } 


  public Command getAutonomousCommand() {
    //return new PathPlannerAuto("MiddleFull"); //IntakeTransferTest
    //return new InstantCommand();
    return autoChooser.getSelected(); // <- Selectable Auto Command  
  }
}
