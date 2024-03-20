package frc.robot.commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends Command {
  
  LEDSubsystem ledSubsystem;
  String pattern;










// do not use














  public LEDCommand(LEDSubsystem ledSubsystem, String pattern) {
    addRequirements(ledSubsystem);
    this.ledSubsystem = ledSubsystem;
    this.pattern = pattern;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //ledSubsystem.setLed(pattern);
    ledSubsystem.setLEDRGB(0, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  } 

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}
