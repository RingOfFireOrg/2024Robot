package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  SwerveSubsystem swerveSubsystem;

  
  @Override
  public void robotInit() {

    // Thread m_visionThread = new Thread(
    // () -> {
    //   UsbCamera camera = CameraServer.startAutomaticCapture();

    //   camera.setResolution(640, 480);
    

    //   CvSink cvSink = CameraServer.getVideo();

    //   CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

    //   Mat mat = new Mat();
      
    //   while (!Thread.interrupted()) {
    //     if (cvSink.grabFrame(mat) == 0) {

    //       outputStream.notifyError(cvSink.getError());

    //       continue;
    //     }

    //     Imgproc.rectangle(
    //         mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);

    //     outputStream.putFrame(mat);
    //   }
    // });
    // m_visionThread.setDaemon(true);
    // m_visionThread.start();

    m_robotContainer = new RobotContainer();


    // // Creates UsbCamera and MjpegServer [1] and connects them
    // UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    // MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
    // mjpegServer1.setSource(usbCamera);

    // // Creates the CvSink and connects it to the UsbCamera
    // CvSink cvSink = new CvSink("opencv_USB Camera 0");
    // cvSink.setSource(usbCamera);

    // // Creates the CvSource and MjpegServer [2] and connects them
    // CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    // MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
    // mjpegServer2.setSource(outputStream);
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }  

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //SmartDashboard.putString("swerveSubsystem Current Command", m_robotContainer.swerveSubsystem.getCurrentCommand().toString());
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {

  }

  

}
