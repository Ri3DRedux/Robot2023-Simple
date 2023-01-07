// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

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
    driveWithJoystick(true);

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
