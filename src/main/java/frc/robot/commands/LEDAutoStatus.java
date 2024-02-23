package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystemStatus;

public class LEDAutoStatus extends Command {
  LEDSubsystem ledSubsystem;
  String pattern;
  Boolean allianceRed = true;
  Supplier<ShooterSubsystemStatus> shooterStatus;

  public LEDAutoStatus(LEDSubsystem ledSubsystem,  Supplier<ShooterSubsystemStatus> shooterStatus) {
    addRequirements(ledSubsystem);
    this.ledSubsystem = ledSubsystem;
    this.shooterStatus = shooterStatus;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.setLed(pattern);
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      allianceRed = (alliance.get() == DriverStation.Alliance.Red);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("pattern current", shooterStatus.get().toString());
    if (shooterStatus.get() == ShooterSubsystemStatus.READY) {
      ledSubsystem.setLEDRGB(0, 255, 0);;
    }
    else if (shooterStatus.get() == ShooterSubsystemStatus.REVING) {
      ledSubsystem.setLed("redChase");
    }
    else {
      if (allianceRed) {
        ledSubsystem.setLed("rainbow");
      }
      else {
        ledSubsystem.setLed("blueGradient");
      }
    }
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
