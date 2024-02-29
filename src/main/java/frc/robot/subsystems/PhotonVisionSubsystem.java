package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {

  public enum LimeLightStatus {
    TRACKING,
    ALIGNING,
    STANDBY,
    IDLE
  }

  private PhotonCamera TomatoSoup;
  private boolean hasTargets;
  private PhotonPipelineResult latestResult;
  private Transform3d bestCameraToTarget;

  LimeLightStatus limeLightStatus = LimeLightStatus.IDLE;

  public final double cameraHeight = Units.inchesToMeters(0);   //Meters
  public final double targetHeight = Units.inchesToMeters(0);   //Meters
  public final double cameraPitch = Units.degreesToRadians(0); // Radians
  
  public PhotonVisionSubsystem() {
    TomatoSoup = new PhotonCamera("TomatoSoup");
  }

  @Override
  public void periodic() {
    latestResult = TomatoSoup.getLatestResult();
    hasTargets = latestResult.hasTargets();
    SmartDashboard.putBoolean("Target Found", hasTargets);

    if (hasTargets) {

      SmartDashboard.putNumber("AprilTag ID", latestResult.getBestTarget().getFiducialId());
      SmartDashboard.putNumber("Pose Ambguiguity ( 0 to 1) ", latestResult.getBestTarget().getPoseAmbiguity());

      bestCameraToTarget = latestResult.getBestTarget().getBestCameraToTarget();

      SmartDashboard.putNumber("X axis Degrees ", Units.radiansToDegrees(bestCameraToTarget.getRotation().getX()));
      SmartDashboard.putNumber("Y Axis Degrees",Units.radiansToDegrees(bestCameraToTarget.getRotation().getY()));
      SmartDashboard.putNumber("Z Axis Degrees",  Units.radiansToDegrees(bestCameraToTarget.getRotation().getZ()));

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

  public void setPipeline(int pipelineIndex) {
    TomatoSoup.setPipelineIndex(pipelineIndex);
  }


}