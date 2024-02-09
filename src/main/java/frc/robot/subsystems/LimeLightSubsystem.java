// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {

  public enum LimeLightStatus {
    TRACKING,
    ALIGNING,
    STANDBY,
    IDLE
  }

  private PhotonCamera TomatoSoup;

  LimeLightStatus limeLightStatus;

  public final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(0); //TODO: find out this numebr 
  public final double TARGET_HEIGHT_METERS = Units.inchesToMeters(0);
  public final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0); 
  
  public LimeLightSubsystem() {
    TomatoSoup = new PhotonCamera("TomatoSoup");
  }

  @Override
  public void periodic() {
    var result = TomatoSoup.getLatestResult();
    boolean hasTargets = result.hasTargets();
    SmartDashboard.putBoolean("Target Found", hasTargets);

    if (hasTargets) {

      SmartDashboard.putNumber("AprilTag ID", result.getBestTarget().getFiducialId());
      SmartDashboard.putNumber("If this isnt above 0.5 smack the limelight", result.getBestTarget().getPoseAmbiguity());

      Transform3d bestCameraToTarget = result.getBestTarget().getBestCameraToTarget();

      SmartDashboard.putNumber("X axis", Units.radiansToDegrees(bestCameraToTarget.getRotation().getX()));
      SmartDashboard.putNumber("Y Axis",Units.radiansToDegrees(bestCameraToTarget.getRotation().getY()));
      SmartDashboard.putNumber("Z Axis",  Units.radiansToDegrees(bestCameraToTarget.getRotation().getZ()));

      SmartDashboard.putNumber("X meters", bestCameraToTarget.getX());
      SmartDashboard.putNumber("Y meters", bestCameraToTarget.getY());
      SmartDashboard.putNumber("Z meters", bestCameraToTarget.getZ());

    }  
  }


  public LimeLightStatus getLimeLightStatusStatus() {
    return limeLightStatus;
  }

  public void setLimeLightStatus(LimeLightStatus newStatus) {
    limeLightStatus = newStatus;
  }

  public PhotonCamera getCamera() {
    return TomatoSoup;
  }

  public BooleanSupplier hasTargetBooleanSupplier() {
    return () -> TomatoSoup.getLatestResult().hasTargets();
  }

  // public void takeInputSnapshot() {
  //   TomatoSoup.takeInputSnapshot();
  // }

  public void onLED()  { 
    TomatoSoup.setLED(VisionLEDMode.kOn);
  }

  public void offLED() { 
    TomatoSoup.setLED(VisionLEDMode.kOff); }


  public void setPipeline(int pipelineIndex) {
    TomatoSoup.setPipelineIndex(pipelineIndex);
  }

  public void setTagMode() {
    setPipeline(0);
    offLED();
  }

  public void setTapeMode() {
    setPipeline(1);
    onLED();
  }
}
