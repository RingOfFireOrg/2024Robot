package frc.robot;

import java.util.HashMap;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AprilTagStrafeLock;
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
import frc.robot.commands.TeleopCommands.SwerveNewJoystick;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KrakenShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem.NoteSesnorStatus;
import frc.robot.subsystems.PivotIntakeSubsystem.PivotSubsystemStatus;
import frc.robot.subsystems.Vision.LimelightHelpers;


public class RobotContainer {
  public LEDSubsystem ledSubsystem = new LEDSubsystem();
  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public PivotIntakeSubsystem pivotIntakeSubsystem = new PivotIntakeSubsystem();
  public KrakenShooterSubsystem krakenShooterSubsystem = new KrakenShooterSubsystem();
  public ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  //public LimeLight limeLightSubsystem = new LimeLight();
  //public PhotonVisionSubsystem photonVisionSubsystem = new PhotonVisionSubsystem();

  private XboxController driverController = new XboxController(OIConstants.driverControllerPort);
  private CommandXboxController operatorController = new CommandXboxController(OIConstants.operatorControllerPort);
  //private XboxController climberController = new XboxController(OIConstants.climberControllerPort); 

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private SendableChooser<String> AutoGenerator2ndNote = new SendableChooser<>();
  private SendableChooser<String> AutoGenerator3rdNote = new SendableChooser<>();
  private SendableChooser<String> AutoGenerator4thNote = new SendableChooser<>();


  HashMap<String, Supplier<Command>> middleHashMap = new HashMap<>();
  ShuffleboardTab codeTestTab = Shuffleboard.getTab("Code Testing");
  ShuffleboardTab generateAutoTab = Shuffleboard.getTab("Generate Auto");
  ShuffleboardTab visionTab = Shuffleboard.getTab("Vision Tab");


  GenericEntry amp_topShooter = codeTestTab
    .add("(A)Top Shooter", -200)
    .getEntry();

  GenericEntry amp_bottomShooter = codeTestTab
    .add("(A)Bottom Shooter", -650)
    .getEntry();

  GenericEntry longShot_topShooter = codeTestTab
    .add("Long Shot Top", -4000)
    .getEntry();

  GenericEntry longShot_bottomShooter = codeTestTab
    .add("Long Shot Bottom", -1500)
    .getEntry();

  GenericEntry midLongShot_topShooter = codeTestTab
    .add("MidLong Shot Top", -3500)
    .getEntry();

  GenericEntry midLongShot_bottomShooter = codeTestTab
    .add("MidLong Shot Bottom", -2500)
    .getEntry();


  public RobotContainer() {
    namedCommands();
    createAutoMapMiddle(middleHashMap);
    populateAutoChooser();
    defaultCommands();
    configureButtonBindings();

  }

  /* Sets default commands for each subsystem */
  private void defaultCommands() {

    ledSubsystem.setDefaultCommand(new LEDTeleOpStatus(
      ledSubsystem, 
      () -> krakenShooterSubsystem.getStatus(),         //Supplier for Shooter Status
      () -> pivotIntakeSubsystem.getIntakeStatus(),    //Supplier for Intake Pivot Status
      () -> intakeSubsystem.getStatus(),               //Supplier for Intake Wheels
      () -> pivotIntakeSubsystem.getNoteSesnorStatus() //Supplier for InfaredSensor
    ));

    swerveSubsystem.setDefaultCommand(new SwerveNewJoystick(
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

      // //Varied Assortment of Buttons to click
      () -> !driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),

      // // Speed Buttons
      () -> driverController.getRawButton(OIConstants.aButton),
      () -> driverController.getRawButton(OIConstants.bButton),
      () -> driverController.getRawButton(OIConstants.xButton),
      () -> driverController.getRawButton(OIConstants.yButton)

    ));

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


    // climberSubsystem.setDefaultCommand(new ClimberTeleop(
    //   climberSubsystem, 
    //   () -> climberController.getLeftY(),
    //   () -> climberController.getRightY()
    // ));
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
    NamedCommands.registerCommand("IntakeIn", new InstantCommand( () -> intakeSubsystem.setMotorFull(0.6), intakeSubsystem));
    NamedCommands.registerCommand("IntakeInSlowly", new InstantCommand( () -> intakeSubsystem.setMotorFull(0.2), intakeSubsystem));

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

    NamedCommands.registerCommand("Auto Pickup Note", 
      new SwerveNoteTrack(swerveSubsystem, pivotIntakeSubsystem.getNoteSesnorStatus())
      .alongWith(intakeSubsystem.setMotorSpeeds(0.7))
      .until(() -> pivotIntakeSubsystem.getNoteSesnorStatus() == NoteSesnorStatus.NOTE_DECTECTED)
      .withTimeout(1)
    );


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

    Command middleGenInst = new InstantCommand();
    middleGenInst.setName(AutoConstants.middleGenName);



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

    /* Autos */
    autoChooser.addOption("2) Middle 2p", new PathPlannerAuto("2) Middle 2p P-MR"));
    autoChooser.addOption("3) Middle 3p Up", new PathPlannerAuto("3) Middle 3p Up"));
    autoChooser.addOption("4) Middle 3p DOWN", new PathPlannerAuto("4) Middle 3p DOWN"));
    autoChooser.addOption("5) Middle 4p", new PathPlannerAuto("5) Middle 4p Middle"));
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
    autoChooser.addOption("Vision Midline", new PathPlannerAuto("_21) Middle 4p Down C3"));
    autoChooser.addOption("Vision Top", new PathPlannerAuto("22)_ Vision Top 3p"));

    
    /* Path tuning */
    autoChooser.addOption("Taxi", new PathPlannerAuto("1meter"));


    autoChooser.addOption("Middle Genearator", middleGenInst);

    populateGenarators(AutoGenerator2ndNote);
    populateGenarators(AutoGenerator3rdNote);
    populateGenarators(AutoGenerator4thNote);

    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(AutoGenerator2ndNote);
    SmartDashboard.putData(AutoGenerator3rdNote);
    SmartDashboard.putData(AutoGenerator4thNote);

    generateAutoTab.add(autoChooser).withPosition(0, 0).withSize(4, 2);
    generateAutoTab.add(AutoGenerator2ndNote).withPosition(0, 2).withSize(4, 2);
    generateAutoTab.add(AutoGenerator3rdNote).withPosition(0, 4).withSize(4, 2);
    generateAutoTab.add(AutoGenerator4thNote).withPosition(0, 6).withSize(4, 2);
  }

  private void createAutoMapMiddle(HashMap<String, Supplier<Command>> hashMap) {
    hashMap.put(null, () -> new InstantCommand());
    hashMap.put("None",() -> new InstantCommand());
    hashMap.put("TopRing",() -> new PathPlannerAuto("Middle - mTopRing"));
    hashMap.put("MiddleRing",() -> new PathPlannerAuto("Middle - mMiddleRing"));
    hashMap.put("BottomRing",() -> new PathPlannerAuto("Middle - mBottomRing"));
    hashMap.put("Centerline 1",() -> new PathPlannerAuto("Middle - CenterLine1"));
    hashMap.put("Centerline 2",() -> new PathPlannerAuto("Middle - CenterLine2"));
    hashMap.put("Centerline 3",() -> new PathPlannerAuto("Middle - CenterLine3"));
    hashMap.put("Centerline 4",() -> new PathPlannerAuto("Middle - CenterLine4"));
    hashMap.put("Centerline 5",() -> new PathPlannerAuto("Middle - CenterLine5"));
  }

  private void populateGenarators(SendableChooser<String> chooser) {
    chooser.setDefaultOption("None", "None");
    chooser.addOption("TopRing", "TopRing");
    chooser.addOption("MiddleRing", "MiddleRing");
    chooser.addOption("BottomRing", "BottomRing");
    chooser.addOption("Centerline 1", "Centerline 1");
    chooser.addOption("Centerline 2", "Centerline 2");
    chooser.addOption("Centerline 3", "Centerline 3");
    chooser.addOption("Centerline 4", "Centerline 4");
    chooser.addOption("Centerline 5", "Centerline 5");
  }





  /* Create button Bindings*/
  private void configureButtonBindings() {

    /* Auto Pickup */
    new JoystickButton(driverController, Constants.OIConstants.leftBumper)
      .whileTrue(new SwerveNoteTrack(swerveSubsystem, pivotIntakeSubsystem.getNoteSesnorStatus())
      .alongWith(intakeSubsystem.setMotorSpeeds(0.8))
      .unless(() -> pivotIntakeSubsystem.getNoteSesnorStatus() == NoteSesnorStatus.NOTE_DECTECTED)
    );

    // new JoystickButton(driverController, Constants.OIConstants.rightBumper)
    //   .whileTrue(new AprilTagStrafeLock(swerveSubsystem, () -> driverController.getLeftY(), () -> driverController.getRightY())
    // );
    
    /* Runs the Shooter at a speed desirable for Amping */
    new JoystickButton(operatorController.getHID(), Constants.OIConstants.xButton)
      .whileTrue(krakenShooterSubsystem.rpmCMD(
        () -> amp_topShooter.getDouble(500),
        () -> amp_bottomShooter.getDouble(500)
      )
    );

    /* Long Shot */
    new JoystickButton(operatorController.getHID(), Constants.OIConstants.yButton)
      .whileTrue(krakenShooterSubsystem.rpmCMD(
        () -> longShot_topShooter.getDouble(-4000),
        () -> longShot_bottomShooter.getDouble(-1500)
      )
    );

    /* Mid Long Shot */
    new JoystickButton(operatorController.getHID(), Constants.OIConstants.aButton)
      .whileTrue(krakenShooterSubsystem.rpmCMD(
        () -> midLongShot_topShooter.getDouble(-3500),
        () -> midLongShot_bottomShooter.getDouble(-2500)
      )
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

    /*auto stop */

    // new JoystickButton(operatorController.getHID(), Constants.OIConstants.leftBumper)
    //   .whileTrue(intakeSubsystem.setMotorSpeeds(0.3)
    //     .alongWith(krakenShooterSubsystem.intakeSpeed())
    //     .onlyWhile(() -> pivotIntakeSubsystem.getNoteSesnorStatus() != NoteSesnorStatus.NOTE_DECTECTED)
    //     .onlyWhile(() -> pivotIntakeSubsystem.getIntakeStatus() != PivotSubsystemStatus.INTAKE_UP)
    //   )
    //   .onFalse(intakeSubsystem.stopIntakeWheel()
    //     .alongWith(krakenShooterSubsystem.stopMotorsCMD())
    // );

    // new JoystickButton(operatorController.getHID(), Constants.OIConstants.rightBumper)
    //   .whileTrue(krakenShooterSubsystem.distanceShot(
    //     photonVisionSubsystem.getMeters(7, 7)
    //   ))
    // ;




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

    // new POVButton(climberController, Constants.OIConstants.dPadUp) 
    //   .whileTrue(new IntakeUp(pivotIntakeSubsystem, 0.4)
    // );

    // new POVButton(climberController, Constants.OIConstants.dPadDown) 
    //   .whileTrue(new IntakeDown(pivotIntakeSubsystem, 0.4)
    // );
    
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
    

    /* Intake Auto Stow using Infared Sensor */
    // operatorController.axisGreaterThan(Constants.OIConstants.leftTrigger, 0.3).and(() -> (pivotIntakeSubsystem.getNoteSesnorStatus() != NoteSesnorStatus.NOTE_DECTECTED))
    //   .onFalse(new IntakeUp(pivotIntakeSubsystem, 0.5)
    //     .alongWith(new IntakeTeleop(intakeSubsystem,() -> 0.0)));
    // ;

  

    

  } 


  

  public Command getAutonomousCommand() {
    /* Run a Specicfic PathPlanner Auto */
    //return new PathPlannerAuto("MiddleFull");

    /* Run no Auto */
    //return new InstantCommand();

    /* Use Sendable Chooser to Select */
    if (autoChooser.getSelected().getName() == AutoConstants.middleGenName) {
      return new ParallelCommandGroup(
        krakenShooterSubsystem.rpmCMD(-3100),
        new SequentialCommandGroup
        (
          new PathPlannerAuto("Middle - Preload").withTimeout(1), //preload
          middleHashMap.get(AutoGenerator2ndNote.getSelected()).get(), //2nd piece
          middleHashMap.get(AutoGenerator3rdNote.getSelected()).get(), //3rd piece
          middleHashMap.get(AutoGenerator4thNote.getSelected()).get() //4th piece
      ));
    }
    return autoChooser.getSelected();  
  }






  /* ---------------------------------------------------------------- */

  /* Flash the limelight when the robot is ready to automatically pickup */
  public void autoFlashPickup() {
    if (
      pivotIntakeSubsystem.getIntakeStatus() == PivotSubsystemStatus.INTAKE_DOWN 
      && pivotIntakeSubsystem.getNoteSesnorStatus() != NoteSesnorStatus.NOTE_DECTECTED
      && LimelightHelpers.getTV(Constants.VisionConstants.NoteCamera) == true
    ) {
      LimelightHelpers.setLEDMode_ForceOn(Constants.VisionConstants.NoteCamera);
    }
    else {
      LimelightHelpers.setLEDMode_ForceOff(Constants.VisionConstants.NoteCamera);
    }
  }

  /* Flash the limelight when desired RPM is reached */
  // public void autoFlashShoot() {
  //   if (pivotIntakeSubsystem.getIntakeStatus() == PivotSubsystemStatus.INTAKE_UP 
  //     && krakenShooterSubsystem.getStatus() == KrakenShooterSubsystemStatus.READY) {
  //     photonVisionSubsystem.flashLED();
  //   }
  //   else {
  //     photonVisionSubsystem.offLED();
  //   }
  // }
  /* ------------------------------------------------------------------- */

}
