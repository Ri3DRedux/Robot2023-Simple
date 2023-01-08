// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashSet;
import java.util.Set;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  //Very simple autonomous - start at a known position, drive forward at speed for time.
  private final Pose2d m_autoStartPose = new Pose2d();
  private final double m_autoDriveTime_sec = 2.0;
  private final double m_autoDriveSpeed_mps = 1.0;
  private Timer m_autoDriveTimer = new Timer();

  Thread m_visionThread;

  @Override
  public void robotInit(){
    SmartDashboard.putBoolean("Use Field Relative", false);
    
    // Only vision in robotInit below here
    m_visionThread =
        new Thread(
            () -> {
              var camera = CameraServer.startAutomaticCapture();

              var cameraWidth = 640;
              var cameraHeight = 480;

              camera.setResolution(cameraWidth, cameraHeight);

              var cvSink = CameraServer.getVideo();
              var outputStream = CameraServer.putVideo("RioApriltags", cameraWidth, cameraHeight);

              var mat = new Mat();
              var grayMat = new Mat();

              var pt0 = new Point();
              var pt1 = new Point();
              var pt2 = new Point();
              var pt3 = new Point();
              var center = new Point();
              var red = new Scalar(0, 0, 255);
              var green = new Scalar(0, 255, 0);

              var aprilTagDetector = new AprilTagDetector();

              var config = aprilTagDetector.getConfig();
              config.quadSigma = 0.8f;
              aprilTagDetector.setConfig(config);

              var quadThreshParams = aprilTagDetector.getQuadThresholdParameters();
              quadThreshParams.minClusterPixels = 250;
              quadThreshParams.criticalAngle *= 5; // default is 10
              quadThreshParams.maxLineFitMSE *= 1.5;
              aprilTagDetector.setQuadThresholdParameters(quadThreshParams);

              aprilTagDetector.addFamily("tag16h5");

              var timer = new Timer();
              timer.start();
              var count = 0;

              while (!Thread.interrupted()) {
                if (cvSink.grabFrame(mat) == 0) {
                  outputStream.notifyError(cvSink.getError());
                  continue;
                }

                Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

                var results = aprilTagDetector.detect(grayMat);

                var set = new HashSet<>();

                for (var result: results) {
                  count += 1;
                  pt0.x = result.getCornerX(0);
                  pt1.x = result.getCornerX(1);
                  pt2.x = result.getCornerX(2);
                  pt3.x = result.getCornerX(3);

                  pt0.y = result.getCornerY(0);
                  pt1.y = result.getCornerY(1);
                  pt2.y = result.getCornerY(2);
                  pt3.y = result.getCornerY(3);

                  center.x = result.getCenterX();
                  center.y = result.getCenterY();

                  set.add(result.getId());

                  Imgproc.line(mat, pt0, pt1, red, 5);
                  Imgproc.line(mat, pt1, pt2, red, 5);
                  Imgproc.line(mat, pt2, pt3, red, 5);
                  Imgproc.line(mat, pt3, pt0, red, 5);

                  Imgproc.circle(mat, center, 4, green);
                  Imgproc.putText(mat, String.valueOf(result.getId()), pt2, Imgproc.FONT_HERSHEY_SIMPLEX, 2, green, 7);

                };

                for (var id : set){
                  System.out.println("Tag: " + String.valueOf(id));
                }

                if (timer.advanceIfElapsed(1.0)){
                  System.out.println("detections per second: " + String.valueOf(count));
                  count = 0;
                }

                outputStream.putFrame(mat);
              }
              aprilTagDetector.close();
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

  }

  @Override
  public void disabledPeriodic(){
    if(RobotController.getUserButton()){
      m_swerve.resetWheelsToForward();
    }

  }

  @Override
  public void autonomousInit(){
    m_swerve.setKnownPosition(m_autoStartPose);
    m_autoDriveTimer.start();

  }

  @Override
  public void autonomousPeriodic() {

    if(!m_autoDriveTimer.hasElapsed(m_autoDriveTime_sec)){
      // Timer not yet expired - drive forward at the right speed
      m_swerve.drive(m_autoDriveSpeed_mps, 0, 0, false);
    } else {
      // Timer expired - stop
      m_swerve.drive(0, 0, 0, false);
    }
  }

  @Override
  public void teleopPeriodic() {

    var frel = SmartDashboard.getBoolean("Use Field Relative", false);
    driveWithJoystick(frel);

    if(m_controller.getAButton()){
      m_swerve.m_gyro.reset();
    }

    // m_swerve.m_backLeft.m_driveMotor.m_motor.set(m_controller.getLeftX());
    // m_swerve.m_backRight.m_driveMotor.m_motor.set(m_controller.getLeftX());
    // m_swerve.m_frontLeft.m_driveMotor.m_motor.set(m_controller.getLeftX());
    // m_swerve.m_frontRight.m_driveMotor.m_motor.set(m_controller.getLeftX());

    // m_swerve.m_backLeft.m_turningMotor.m_motor.set(m_controller.getRightX());
    // m_swerve.m_backRight.m_turningMotor.m_motor.set(m_controller.getRightX());
    // m_swerve.m_frontLeft.m_turningMotor.m_motor.set(m_controller.getRightX());
    // m_swerve.m_frontRight.m_turningMotor.m_motor.set(m_controller.getRightX());

  }

  @Override
  public void robotPeriodic(){
    m_swerve.updateOdometry();
  }

  private void driveWithJoystick(boolean fieldRelative) {
    //the xbox controller we use has around a 5% center error
    var xDead = MathUtil.applyDeadband(m_controller.getLeftY(), 0.05);
    var yDead = MathUtil.applyDeadband(m_controller.getLeftX(), 0.05);
    var rotDead = MathUtil.applyDeadband(m_controller.getRightX(), 0.05);
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xspeedLimiter.calculate(xDead) * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(yDead) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(rotDead) * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
