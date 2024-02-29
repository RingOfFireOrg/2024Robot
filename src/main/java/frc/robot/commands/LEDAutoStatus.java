package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem.PivotSubsystemStatus;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystemStatus;

public class LEDAutoStatus extends Command {
  LEDSubsystem ledSubsystem;
  String pattern;
  Boolean allianceRed = true;
  Supplier<ShooterSubsystemStatus> shooterStatus;
  Supplier<PivotSubsystemStatus> pivotIntakeStatus;

  public LEDAutoStatus(LEDSubsystem ledSubsystem,  Supplier<ShooterSubsystemStatus> shooterStatus, Supplier<PivotSubsystemStatus> pivotIntakeStatus) {
    addRequirements(ledSubsystem);
    this.ledSubsystem = ledSubsystem;
    this.shooterStatus = shooterStatus;
    this.pivotIntakeStatus = pivotIntakeStatus;
  }

  @Override
  public void initialize() {
    ledSubsystem.setLed(pattern);
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      allianceRed = (alliance.get() == DriverStation.Alliance.Red);
    }

  }

  @Override
  public void execute() {
    if (pivotIntakeStatus.get() == PivotSubsystemStatus.INTAKE_DOWN) {
      //orange?
    }
    if (shooterStatus.get() == ShooterSubsystemStatus.READY) {
      ledSubsystem.setLEDRGB(0, 255, 0);
      //Leave as is(?)
    }
    else if (shooterStatus.get() == ShooterSubsystemStatus.REVING) {
      ledSubsystem.setLed("redChase");
      //Make a RED pattern that is pointing Upwards
    }
    else if (shooterStatus.get() == ShooterSubsystemStatus.REVERSE) {
      ledSubsystem.setLed("redChase");
      //Make a RED pattern that is pointing downwards
    }
    else {
      // Alliance Color LEDS
      if (allianceRed) {
        //TODO: make Orange Gradient pattern - NEEDS to be distcint from red so it does not get confused with shooter, but also from when intake is down
        ledSubsystem.setLEDRGB(255, 0, 0);
        //ledSubsystem.setLed("redGradient");
        //ledSubsystem.setLEDRGB(Constants.LEDConstants.pyrotechOrange[0], Constants.LEDConstants.pyrotechOrange[1], Constants.LEDConstants.pyrotechOrange[2]);
      }
      else {
        //TODO: make an actual blue gradient
        ledSubsystem.setLed("blueGradient");
      }
    }
  }






  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() { return false; }

  @Override
  public boolean runsWhenDisabled(){
    // Is this legal?
    return true;
  }
}
