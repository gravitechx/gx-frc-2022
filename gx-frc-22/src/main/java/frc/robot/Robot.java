// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final int RESOLUTION_X = 720;
  private static final int RESOLUTION_Y = 480;    

  // A new thread for the camera.
  Thread m_visionThread;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Simple camera code. This line of code works.
    // Resolution: more than 240 is a risk. 480p is the highest we can go.

    // https://docs.wpilib.org/en/stable/docs/software/vision-processing/introduction/cameraserver-class.html
    // Creates UsbCamera and MkpegServer [1], connects them.
    CameraServer.startAutomaticCapture();
    // // Creates the CvSink and connects to UsbCamera
    // CvSink cvSink = CameraServer.getVideo();
    // // Creates the CvSource and MjpegServer [2] and connects them.
    // CvSource outputStream = CameraServer.putVideo("Stream", RESOLUTION_X, RESOLUTION_Y);


    // We can use CameraServer as the constructor because .startAutomaticCapture returns a UsbCamera?
    // UsbCamera camera = CameraServer.startAutomaticCapture();
    // camera.setResolution(RESOLUTION_X, RESOLUTION_Y);


    // More complex camera code. Camera code (including above) are from https://docs.wpilib.org/en/stable/docs/software/vision-processing/roborio/using-the-cameraserver-on-the-roborio.html
    // A lot of lagging.
    // m_visionThread = 
    //   new Thread(
    //     () -> {

    //       // Get USB Camera from CameraServer.
    //       UsbCamera camera = CameraServer.startAutomaticCapture();
    //       // Resolution
    //       camera.setResolution(RESOLUTION_X, RESOLUTION_Y);

    //       // Get a CvSink.
    //       CvSink cvSink = CameraServer.getVideo();
    //       // Open CvSource, which gives images to the dashboard.
    //       CvSource outputStream = CameraServer.putVideo("Rectangle", RESOLUTION_X, RESOLUTION_Y);

    //       Mat mat = new Mat();

    //       // This can't be true, as the program will not close if it is. This is meant
    //       while (!Thread.interrupted()) {

    //         // CvSink has to take a frame, put it in source mat.

    //         // Errors.
    //         if (cvSink.grabFrame(mat) == 0) {
    //           // Send stream the error.
    //           outputStream.notifyError(cvSink.getError());

    //           // skip current iteration
    //           continue;
    //         }
      
    //         // Put rectangle in the picture.
    //         Imgproc.rectangle(mat, new Point(100,100), new Point(400, 400), new Scalar(255, 255, 255), 5);
    //         // Place output stream a new image, now with a picture.
    //         outputStream.putFrame(mat);
    //       }
    //     });
    //   m_visionThread.setDaemon(true);
    //   m_visionThread.start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

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
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
