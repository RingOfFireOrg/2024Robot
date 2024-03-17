package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystemStatus;
import frc.robot.subsystems.KrakenShooterSubsystem.KrakenShooterSubsystemStatus;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem.PivotSubsystemStatus;

public class LEDAutoStatus extends Command {
  LEDSubsystem ledSubsystem;
  String pattern;
  Boolean allianceRed = true;
  Supplier<KrakenShooterSubsystemStatus> shooterStatus;
  Supplier<PivotSubsystemStatus> pivotIntakeStatus;
  Supplier<IntakeSubsystemStatus> intakeStatus;

  public LEDAutoStatus(LEDSubsystem ledSubsystem,  Supplier<KrakenShooterSubsystemStatus> shooterStatus, Supplier<PivotSubsystemStatus> pivotIntakeStatus, Supplier<IntakeSubsystemStatus> intakeStatus) {
    addRequirements(ledSubsystem);
    this.ledSubsystem = ledSubsystem;
    this.shooterStatus = shooterStatus;
    this.pivotIntakeStatus = pivotIntakeStatus;
    this.intakeStatus = intakeStatus;

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


    if (pivotIntakeStatus.get() == PivotSubsystemStatus.INTAKE_DOWN && intakeStatus.get() == IntakeSubsystemStatus.INTAKE_IN) {
      ledSubsystem.redMoveSplit_REVERSE();
    }
    else if (pivotIntakeStatus.get() != PivotSubsystemStatus.INTAKE_UP) {
      ledSubsystem.setLEDRGB(255, 0, 0);
    }
    else {
      // Alliance Color LEDS
      if (allianceRed) {
        ledSubsystem.shiftingOrange_BAR();
      }
      else {
        ledSubsystem.blueGradient();
      }
    }
  }






  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() { return false; }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
