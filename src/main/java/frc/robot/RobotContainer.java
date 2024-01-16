package frc.robot;


import frc.robot.commands.TeleopCommands.LEDCommand;
import frc.robot.commands.TeleopCommands.SwerveJoystickCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;




public class RobotContainer {
  LEDSubsystem ledSubsystem = new LEDSubsystem();

  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  //public LimeLight limeLightSubsystem = new LimeLight();


  private final XboxController driverController = new XboxController(OIConstants.driverControllerPort);
  private final XboxController operatorController = new XboxController(OIConstants.operatorControllerPort);

  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  //private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    // NEW CODE
    //autoChooser = AutoBuilder.buildAutoChooser(); // this will build the auto chooser with no default command set in
    //autoChooser = AutoBuilder.buildAutoChooser("Default Auto Name"); //Use this to set a default auto in
    //SmartDashboard.putData("Auto Chooser", autoChooser);


    ledSubsystem.setDefaultCommand(new LEDCommand(ledSubsystem));


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
    return new PathPlannerAuto("Example Auto");
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
