package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {












// MAKE NEW LIMELIGHT SUBSYSTEM











  public LimeLight( ) {


  } 
  public double[] getVisionVals() {
    // https://docs.limelightvision.io/en/latest/networktables_api.html
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // Horizontal Offset From Crosshair To Target (-29.8 to 29.8deg)
    double x = table.getEntry("tx").getDouble(0.0);

    // Vertical Offset From Crosshair To Target (-24.85 to 24.85deg)
    double y = table.getEntry("ty").getDouble(0.0);

    // Valid target in vision (0 or 1)
    double v = table.getEntry("ty").getDouble(0.0);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    // post to smart dashboard
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightY", y);

    double[] arr = { x, y, v };
    return arr;
  }
  
  // CameraServer limelight;
  // limelight = new HttpCamera("limelight", "http://10.34.59.98:5801/", HttpCameraKind.kMJPGStreamer);
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void limeLightDashboardVals() {
    // https://docs.limelightvision.io/en/latest/networktables_api.html
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // Horizontal Offset From Crosshair To Target (-29.8 to 29.8deg)
    double x = table.getEntry("tx").getDouble(0.0);

    // Vertical Offset From Crosshair To Target (-24.85 to 24.85deg)
    double y = table.getEntry("ty").getDouble(0.0);

    // Valid target in vision (0 or 1)
    double v = table.getEntry("ty").getDouble(0.0);

    // post to smart dashboard
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightY", v);

  }

 
}