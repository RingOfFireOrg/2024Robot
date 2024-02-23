package frc.robot;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AmpSpeeds;
import frc.robot.commands.LEDAutoStatus;
import frc.robot.commands.TeleopCommands.ClimberTeleop;
import frc.robot.commands.TeleopCommands.IntakePivotTeleop;
import frc.robot.commands.TeleopCommands.IntakeTeleop;
import frc.robot.commands.TeleopCommands.LEDCommand;
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

  LEDSubsystem ledSubsystem = new LEDSubsystem();

  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  //public PivotShooterSubsystem pivotSubsystem = new PivotShooterSubsystem();
  public ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public PivotIntakeSubsystem pivotIntakeSubsystem = new PivotIntakeSubsystem();

  public ClimberSubsystem climberSubsystem = new ClimberSubsystem();




  //public LimeLight limeLightSubsystem = new LimeLight();


  XboxController driverController = new XboxController(OIConstants.driverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.operatorControllerPort);
  XboxController climberController = new XboxController(2);

  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  //private final SendableChooser<Command> autoChooser;


  public RobotContainer() {

    // NamedCommands.registerCommand("command1", new InstantCommand());
    // NamedCommands.registerCommand("command2", new InstantCommand());
    // NamedCommands.registerCommand("command3", new InstantCommand());

    // autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Auto Chooser", autoChooser);




    //ledSubsystem.setDefaultCommand(new LEDCommand(ledSubsystem,"blueGradient"));
    ledSubsystem.setDefaultCommand(new LEDAutoStatus(ledSubsystem, () -> shooterSubsystem.getStatus()));
    //ledSubsystem.


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
    // //new JoystickButton(driverController, 4).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    // new JoystickButton(driverController, Constants.OIConstants.backButton).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    // JoystickButton hi = new JoystickButton(operatorController, Constants.OIConstants.aButton);
    // hi.whileTrue(new InstantCommand(() ->shooterSubsystem.ampSpeeds()));
    // //.onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    // driverController.getBackButton().whileTrue();

    new JoystickButton(operatorController, Constants.OIConstants.aButton).whileTrue(new AmpSpeeds(shooterSubsystem));
    new JoystickButton(driverController, Constants.OIConstants.backButton).whileTrue(new RunCommand(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));


    //operatorController.getAButton().whileTrue(new InstantCommand(() ->shooterSubsystem.ampSpeeds()));


  } 


  public Command getAutonomousCommand() {

    // NEW CODE - to run an auto, run it like this, however a builder may be required to run commands inside of it, 
    // alternative is to make a sequential group, which might be a  lil iffy
    return new PathPlannerAuto("test");
    
    //return new InstantCommand();

    //return autoChooser.getSelected();  <- Selectable Auto Command

  }
}
