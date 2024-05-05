package frc.robot.subsystems.Vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PhotonVisionSubsystem extends SubsystemBase {

  public enum CameraStatus {
    TRACKING,
    ALIGNING,
    STANDBY,
    IDLE
  }

  enum TagLocations {
    test1
  }
  
  private double lastEstTimestamp = 0;

  private PhotonCamera TomatoSoup;
  private boolean hasTargets;
  private PhotonPipelineResult latestResult;
  private Transform3d bestCameraToTarget;
  PhotonPoseEstimator photonPoseEstimator;

  CameraStatus cameraStatus = CameraStatus.IDLE;

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


  public final double cameraHeight = Units.inchesToMeters(0);   // Put inches
  public final double targetHeight = Units.inchesToMeters(0);   // Put Inches
  public final double cameraPitch = Units.degreesToRadians(0); // Put Radians


  Transform3d robotToCam = 
    new Transform3d(
      new Translation3d(0.5, 0.0, 0.5), 
      new Rotation3d(0,0,0)
    ); 
      //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  ShuffleboardTab codeTestTab = Shuffleboard.getTab("Code Testing");


  public PhotonVisionSubsystem() {
    TomatoSoup = new PhotonCamera("PT_BackCam");
    photonPoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout, 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
      TomatoSoup, 
      robotToCam
    );
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }

  @Override
  public void periodic() {
    latestResult = TomatoSoup.getLatestResult();
    hasTargets = latestResult.hasTargets();
  
    SmartDashboard.putBoolean("Target Found", hasTargets);
    SmartDashboard.putString("LED Mode",TomatoSoup.getLEDMode().name());

    if (hasTargets) {
      SmartDashboard.putNumber("AprilTag ID", latestResult.getBestTarget().getFiducialId());
      SmartDashboard.putBoolean("Speaker Tag", latestResult.getBestTarget().getFiducialId() == 7);
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


  // public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
  //   var visionEst = photonPoseEstimator.update();
  //   double latestTimestamp = TomatoSoup.getLatestResult().getTimestampSeconds();
  //   boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
  //   if (newResult) {
  //     lastEstTimestamp = latestTimestamp;
  //   }
  //   return visionEst;
  // }




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

  public boolean hasTargets(){
    return TomatoSoup.getLatestResult().hasTargets();
  }
  
  public PhotonTrackedTarget getBestTarget(){
    if (hasTargets() == true) {
      return TomatoSoup.getLatestResult().getBestTarget();
    }
    else{
     return null;
    }
  }

  public int targetID(PhotonTrackedTarget photonTrackedTarget){
    return photonTrackedTarget.getFiducialId();
  }

  public boolean confirmID(int trackedId, int desiredID) {
    return trackedId == desiredID;
  }

  public double getMeters(int redID, int blueID) {
    codeTestTab.addBoolean("getmeters_Has target",() -> TomatoSoup.getLatestResult().hasTargets());
    if(TomatoSoup.getLatestResult().hasTargets() == true) {
      PhotonTrackedTarget bestTarget = getBestTarget();
      codeTestTab.addDouble("getmeters_Target ID", () -> targetID(bestTarget));
      if(targetID(bestTarget) == redID || targetID(bestTarget) == blueID) {
        codeTestTab.addDouble("getmeters_Return Value", () -> Math.hypot(bestTarget.getBestCameraToTarget().getX(),bestTarget.getBestCameraToTarget().getY()));
        return Math.hypot(bestTarget.getBestCameraToTarget().getX(),bestTarget.getBestCameraToTarget().getY()) ;
      }
    }
    return 999;
  }

  public double getDistancefromPitch(PhotonTrackedTarget bestTarget) {
    return PhotonUtils.calculateDistanceToTargetMeters(
      cameraHeight,
      targetHeight,
      cameraPitch,
      Units.degreesToRadians(bestTarget.getPitch()
    ));
  }


  public void flashLED() {
    TomatoSoup.setLED(VisionLEDMode.kOn);
    
  }

  public void offLED() {
    TomatoSoup.setLED(VisionLEDMode.kOff);
  }
}