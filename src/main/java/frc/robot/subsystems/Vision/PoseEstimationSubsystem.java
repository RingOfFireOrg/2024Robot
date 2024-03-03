// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class PoseEstimationSubsystem extends SubsystemBase {

  private final PhotonCamera photonCamera;
  private final SwerveSubsystem swerveSubsystem;
  private final AprilTagFieldLayout aprilTagFieldLayout;
  

  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;


  public PoseEstimationSubsystem(PhotonCamera photonCamera, SwerveSubsystem swerveSubsystem) {
    this.photonCamera = photonCamera;
    this.swerveSubsystem = swerveSubsystem;
    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      layout.setOrigin(alliance.get() == DriverStation.Alliance.Blue ?
          OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.aprilTagFieldLayout = layout;
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    poseEstimator =  new SwerveDrivePoseEstimator(
      Constants.DriveConstants.kDriveKinematics,
      swerveSubsystem.getRotation2d(),
      swerveSubsystem.getSwerveModulePosition(),
      new Pose2d(),
      stateStdDevs,
      visionMeasurementStdDevs
    );
    
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);


  }

  @Override
  public void periodic() {
    // Update pose estimator with the best visible target
    var pipelineResult = photonCamera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
      if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
        var targetPose = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(VisionConstants.CAMERA_TO_ROBOT);
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
      }
    }
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
      swerveSubsystem.getRotation2d(),
      swerveSubsystem.getSwerveModulePosition());

    field2d.setRobotPose(getCurrentPose());    
  }
  
  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      swerveSubsystem.getRotation2d(),
      swerveSubsystem.getSwerveModulePosition(),
      newPose);
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }
  public static class VisionConstants {
    //Physical location of the camera on the robot, relative to the center of the robot.
    public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d());
    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
  }
}
