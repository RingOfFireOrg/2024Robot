package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystemStatus;
import frc.robot.subsystems.KrakenShooterSubsystem.KrakenShooterSubsystemStatus;
import frc.robot.subsystems.PivotIntakeSubsystem.PivotSubsystemStatus;
import frc.robot.subsystems.PivotIntakeSubsystem.NoteSesnorStatus;

public class LEDTeleOpStatus extends Command {
  LEDSubsystem ledSubsystem;
  String pattern;
  Boolean allianceRed = true;
  Supplier<KrakenShooterSubsystemStatus> shooterStatus;
  Supplier<PivotSubsystemStatus> pivotIntakeStatus;
  Supplier<IntakeSubsystemStatus> intakeStatus;
  Supplier<NoteSesnorStatus> noteSensorStatus;
  
  public LEDTeleOpStatus(
    LEDSubsystem ledSubsystem,  
    Supplier<KrakenShooterSubsystemStatus> shooterStatus, 
    Supplier<PivotSubsystemStatus> pivotIntakeStatus, 
    Supplier<IntakeSubsystemStatus> intakeStatus,
    Supplier<NoteSesnorStatus> noteSensorStatus
    ) {
    addRequirements(ledSubsystem);
    this.ledSubsystem = ledSubsystem;
    this.shooterStatus = shooterStatus;
    this.pivotIntakeStatus = pivotIntakeStatus;
    this.intakeStatus = intakeStatus;
    this.noteSensorStatus = noteSensorStatus;
  }

  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      allianceRed = (alliance.get() == DriverStation.Alliance.Red);
    }

  }

  @Override
  public void execute() {

    if (shooterStatus.get() == KrakenShooterSubsystemStatus.READY) {
      ledSubsystem.setLEDRGB(0, 255, 0);
    }
    else if (shooterStatus.get() == KrakenShooterSubsystemStatus.REVING) {
      ledSubsystem.redMoveSplit();
      ledSubsystem.setLEDRGB_TOP(0, 0, 0);

      //Make a RED pattern that is pointing Upwards
    }
    else if (shooterStatus.get() == KrakenShooterSubsystemStatus.REVERSE) {
      ledSubsystem.redMoveSplit_REVERSE();
      ledSubsystem.setLEDRGB_TOP(0, 0, 0);

      //ledSubsystem.setLEDRGB(0, 255, 0);

      //ledSubsystem.redMoveSplit();
      //Make a RED pattern that is pointing downwards
    }
    else if (pivotIntakeStatus.get() == PivotSubsystemStatus.INTAKE_DOWN && intakeStatus.get() == IntakeSubsystemStatus.INTAKE_IN && noteSensorStatus.get() == NoteSesnorStatus.NOTE_DECTECTED) {
      ledSubsystem.setLEDRGB(0, 255, 0);
    }
    else if (pivotIntakeStatus.get() == PivotSubsystemStatus.INTAKE_DOWN && intakeStatus.get() == IntakeSubsystemStatus.INTAKE_IN && noteSensorStatus.get() != NoteSesnorStatus.NOTE_DECTECTED) {
      ledSubsystem.redMoveSplit();
      ledSubsystem.setLEDRGB_TOP(0, 0, 0);

      //ledSubsystem.setLEDRGB(0, 255, 0);

    }
    else if (pivotIntakeStatus.get() == PivotSubsystemStatus.INTAKE_DOWN && noteSensorStatus.get() != NoteSesnorStatus.NOTE_DECTECTED) {
      ledSubsystem.setLEDRGB(255, 0, 0);
      //ledSubsystem.setLEDRGB_TOP(0, 0, 0);

    }
    else {
      if (allianceRed) {
        //ledSubsystem.shiftingOrange_BAR();
        ledSubsystem.setLEDRGB(Constants.LEDConstants.PyroTechOrange[0], Constants.LEDConstants.PyroTechOrange[1], Constants.LEDConstants.PyroTechOrange[2]);
      }
      else {
        //ledSubsystem.blueGradient();
        ledSubsystem.setLEDRGB(Constants.LEDConstants.CryoTechBlue[0], Constants.LEDConstants.CryoTechBlue[1], Constants.LEDConstants.CryoTechBlue[2]);

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
