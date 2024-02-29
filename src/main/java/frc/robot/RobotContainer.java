package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AmpSpeedsRaw;
import frc.robot.commands.LEDAutoStatus;
import frc.robot.commands.OTFPathGen;
import frc.robot.commands.TurnToClimb;
import frc.robot.commands.AutoCommands.IntakeDown;
import frc.robot.commands.AutoCommands.IntakeUp;
import frc.robot.commands.AutoCommands.ShootCMD;
import frc.robot.commands.AutoCommands.TransferCMD;
import frc.robot.commands.TeleopCommands.ClimberTeleop;
import frc.robot.commands.TeleopCommands.IntakePivotTeleop;
import frc.robot.commands.TeleopCommands.IntakeTeleop;
import frc.robot.commands.TeleopCommands.ShooterTeleop;
import frc.robot.commands.TeleopCommands.SwerveJoystickCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;




public class RobotContainer {
  //private final SendableChooser<Command> autoChooser;

  public LEDSubsystem ledSubsystem = new LEDSubsystem();

  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public PivotIntakeSubsystem pivotIntakeSubsystem = new PivotIntakeSubsystem();
  //public PivotShooterSubsystem pivotSubsystem = new PivotShooterSubsystem(); // removed until shooter can pivot
  public ShooterSubsystem shooterSubsystem = new ShooterSubsystem();




  //public LimeLight limeLightSubsystem = new LimeLight();


  XboxController driverController = new XboxController(OIConstants.driverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.operatorControllerPort);
  XboxController climberController = new XboxController(2); //TODO: remove?

  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  //private final SendableChooser<Command> autoChooser;


  public RobotContainer() {

    NamedCommands.registerCommand("Shoot", new ShootCMD(shooterSubsystem));
    NamedCommands.registerCommand("Transfer", new TransferCMD(intakeSubsystem));

    NamedCommands.registerCommand("ShootOn", new InstantCommand( () -> shooterSubsystem.setMotor(0.6), shooterSubsystem));
    NamedCommands.registerCommand("ShootIdle", new InstantCommand( () -> shooterSubsystem.setMotor(0.5), shooterSubsystem));
    NamedCommands.registerCommand("ShootOff", new InstantCommand( () -> shooterSubsystem.setMotor(0), shooterSubsystem));

    NamedCommands.registerCommand("IntakeIn", new InstantCommand( () -> intakeSubsystem.setMotor(0.5), intakeSubsystem));
    NamedCommands.registerCommand("IntakeOut", new InstantCommand( () -> intakeSubsystem.setMotor(-0.5), intakeSubsystem));
    NamedCommands.registerCommand("IntakeOff", new InstantCommand( () -> intakeSubsystem.setMotor(0), intakeSubsystem));


    NamedCommands.registerCommand("IntakeUp", new IntakeUp(pivotIntakeSubsystem));
    NamedCommands.registerCommand("IntakeDown", new IntakeDown(pivotIntakeSubsystem));


    // autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Auto Chooser", autoChooser);




    //ledSubsystem.setDefaultCommand(new LEDCommand(ledSubsystem,"blueGradient")); <- Nonchaning led command
    ledSubsystem.setDefaultCommand(new LEDAutoStatus(
      ledSubsystem, 
      () -> shooterSubsystem.getStatus(),          //Supplier for Shooter Status
      () -> pivotIntakeSubsystem.getIntakeStatus() //Supplier for Intake Pivot Status
      
    )); // <- Changes with status updates from attachemnts


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

    shooterSubsystem.setDefaultCommand(new ShooterTeleop(
      shooterSubsystem, 
      () -> (operatorController.getLeftTriggerAxis() - operatorController.getRightTriggerAxis()) 
    ));

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
     


    configureButtonBindings();

  }

  private void configureButtonBindings() {

    // This no longer works(?) -> new JoystickButton(driverController, Constants.OIConstants.backButton).onTrue(new RunCommand(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));
    // new JoystickButton()
    // driveStick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    new JoystickButton(driverController, Constants.OIConstants.rightBumper).whileTrue(new TurnToClimb(swerveSubsystem));
    new JoystickButton(driverController, Constants.OIConstants.leftBumper).whileTrue(new OTFPathGen(swerveSubsystem));
    new JoystickButton(operatorController, Constants.OIConstants.aButton).whileTrue(new AmpSpeedsRaw(shooterSubsystem));
    //operatorController.getAButton().whileTrue(new InstantCommand(() ->shooterSubsystem.ampSpeeds()));


  } 


  public Command getAutonomousCommand() {
    return new PathPlannerAuto("4pMid[Shoot]"); //IntakeTransferTest
    //return new InstantCommand();
    //return autoChooser.getSelected();  <- Selectable Auto Command

  }
}
