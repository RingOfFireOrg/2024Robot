package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTeleop extends Command {
  
  ClimberSubsystem climberSubsystem;
  Supplier<Double> Leftspeed;
  Supplier<Double> Rightspeed;
    
  public ClimberTeleop(ClimberSubsystem climberSubsystem, Supplier<Double> Leftspeed, Supplier<Double> Rightspeed) {
    addRequirements(climberSubsystem);
    this.Leftspeed = Leftspeed;              
    this.Rightspeed = Rightspeed;                                         
    this.climberSubsystem = climberSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {;
    climberSubsystem.setBothMotors(Leftspeed.get(), Rightspeed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
