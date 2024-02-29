// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class OTFPathGen extends Command {
  SwerveSubsystem swerveSubsystem;
  public OTFPathGen(SwerveSubsystem swerveSubsystem) {
    addRequirements(swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d currentPose = swerveSubsystem.getPose();
    Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d(Units.radiansToDegrees(90)));

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
    PathPlannerPath path = new PathPlannerPath(
      bezierPoints, 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ),  
      new GoalEndState(0.0, currentPose.getRotation())
    );

    // Prevent this path from being flipped on the red alliance, since the given positions are already correct
    path.preventFlipping = true;

    AutoBuilder.followPath(path).schedule();
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
