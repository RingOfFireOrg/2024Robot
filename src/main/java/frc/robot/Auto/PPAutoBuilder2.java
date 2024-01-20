// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PPAutoBuilder2 extends SequentialCommandGroup {
  public PPAutoBuilder2
  (
    SwerveSubsystem driveSubsystem,
    String autoPath,
    double maxVel,
    double maxAccel    
  ) 
  {
    PathPlannerPath path = PathPlannerPath.fromPathFile(autoPath);



    new FollowPathHolonomic(
      path,
      driveSubsystem::getPose, 
      driveSubsystem::getRobotRelativeSpeeds, 
      driveSubsystem::driveRobotRelative, 


      new HolonomicPathFollowerConfig
      (
        new PIDConstants(7, 0.0, 0), // Translation PIDS -> move to constants file at some point
        //TODO: find actual numbers for this
        new PIDConstants(5, 0.0, 0.02), // Rotation PIDS -> move to constants at some point
        4.5,
        0.69, //TODO: find out what this means
        new ReplanningConfig()
      ), 



      () -> {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
      }, 
      driveSubsystem
    );
















  }

}
