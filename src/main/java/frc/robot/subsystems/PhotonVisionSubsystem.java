package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class PhotonVisionSubsystem extends SubsystemBase {

  public enum CameraStatus {
    TRACKING,
    ALIGNING,
    STANDBY,
    IDLE
  }
  private double lastEstTimestamp = 0;

  private PhotonCamera TomatoSoup;
  private boolean hasTargets;
  private PhotonPipelineResult latestResult;
  private Transform3d bestCameraToTarget;
  PhotonPoseEstimator photonPoseEstimator;

  CameraStatus cameraStatus = CameraStatus.IDLE;

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


  public final double cameraHeight = Units.inchesToMeters(0);   //Meters
  public final double targetHeight = Units.inchesToMeters(0);   //Meters
  public final double cameraPitch = Units.degreesToRadians(0); // Radians


  

  public PhotonVisionSubsystem() {
    TomatoSoup = new PhotonCamera("TomatoSoup");
    photonPoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, TomatoSoup, new Transform3d()
    );
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
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


  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonPoseEstimator.update();
    double latestTimestamp = TomatoSoup.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (newResult) {
      lastEstTimestamp = latestTimestamp;
    }
    return visionEst;
  }




  // public void update(double leftDist, double rightDist, Rotation2d rotation2d) {
  //     photonPoseEstimator.update(rotation2d, leftDist, rightDist);

  //     var res = TomatoSoup.getLatestResult();
  //     if (res.hasTargets()) {
  //         var imageCaptureTime = res.getTimestampSeconds();
  //         var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
  //         var camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
  //         m_poseEstimator.addVisionMeasurement(
  //                 camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime);
  //     }
  // }

  public CameraStatus getLimeLightStatusStatus() {
    return cameraStatus;
  }

  public void setLimeLightStatus(CameraStatus newStatus) {
    cameraStatus = newStatus;
  }

  public PhotonCamera getCamera() {
    return TomatoSoup;
  }

  public void setPipeline(int pipelineIndex) {
    TomatoSoup.setPipelineIndex(pipelineIndex);
  }


}