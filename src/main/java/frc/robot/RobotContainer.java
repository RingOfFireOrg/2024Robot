package frc.robot;


import frc.robot.commands.TeleopCommands.LEDCommand;
import frc.robot.commands.TeleopCommands.SwerveJoystickCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.PPAutoBuilder2;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;




public class RobotContainer {
  //private final SendableChooser<Command> autoChooser;

  LEDSubsystem ledSubsystem = new LEDSubsystem();

  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  //public LimeLight limeLightSubsystem = new LimeLight();


  private final XboxController driverController = new XboxController(OIConstants.driverControllerPort);
  private final XboxController operatorController = new XboxController(OIConstants.operatorControllerPort);

  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  //private final SendableChooser<Command> autoChooser;


  public RobotContainer() {

    // NamedCommands.registerCommand("command1", new InstantCommand());
    // NamedCommands.registerCommand("command2", new InstantCommand());
    // NamedCommands.registerCommand("command3", new InstantCommand());

    // autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Auto Chooser", autoChooser);




    ledSubsystem.setDefaultCommand(new LEDCommand(ledSubsystem,"blueGradient"));
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
     
    configureButtonBindings();

  }

  private void configureButtonBindings() {
    //new JoystickButton(driverController, 2).whenPressed(() -> swerveSubsystem.zeroHeading());

  }


  public Command getAutonomousCommand() {

    // NEW CODE - to run an auto, run it like this, however a builder may be required to run commands inside of it, 
    // alternative is to make a sequential group, which might be a  lil iffy
    //return new PathPlannerAuto("Example Auto");

    
    // return new PPAutoBuilder2
    // (    
    //   swerveSubsystem,
    //   "1meter",
    //   1,
    //   1   
    // );

    return new PathPlannerAuto("Rotation");


    //return autoChooser.getSelected();  <- Selectable Auto Command


    // return new SequentialCommandGroup 
    // ( 
    //   //new HighCubeDrop(armSubsystem, outtakeTransferSubsystem, pistonIntakeSubsystem, swerveSubsystem)
    //   //new FollowTrajectoryPathPlanner(swerveSubsystem, "3meter7", true,1,1,false),
    //   //new FollowTrajectoryPathPlanner(swerveSubsystem, "PIDTesting5", false,1,1,false),
    //   //new FollowTrajectoryPathPlanner(swerveSubsystem, "PIDTesting6", false,1,1,false),
    //   //new PPSwerveAutoBuilder(swerveSubsystem, armSubsystem, outtakeTransferSubsystem, pistonIntakeSubsystem, "THORAUTO1", 0, 0)
      
    //   new PIDAutoBalancer(swerveSubsystem)
    //   //new ReversePIDAutoBalancer(swerveSubsystem)
    //   //new newBalance(swerveSubsystem)
    //   //new HighCubeDrop(armSubsystem, outtakeTransferSubsystem, pistonIntakeSubsystem, swerveSubsystem)

    // );
   
  }
}
