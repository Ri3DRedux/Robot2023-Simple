// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class Robot extends TimedRobot {
  private final CommandXboxController m_driver_controller = new CommandXboxController(0);
  private final CommandXboxController m_operator_controller = new CommandXboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();
  private final Elevator m_elevator = new Elevator();

  private final Pose2d m_autoStartPose = new Pose2d();

  @Override
  public void robotInit() {
    CommandScheduler.getInstance().registerSubsystem(m_elevator);
    configureButtonBindings();

    SmartDashboard.putBoolean("Use Field Relative", false);
  }

  @Override
  public void disabledPeriodic() {
    if (RobotController.getUserButton()) {
      m_swerve.resetWheelsToForward();
    }

    m_elevator.m_encoder.setPosition(m_elevator.getToFHeight());

  }

  @Override
  public void autonomousInit() {
    m_swerve.setKnownPosition(m_autoStartPose);

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  public void configureButtonBindings() {

    m_driver_controller.a().onTrue(
        Commands.runOnce(() -> {
          m_swerve.m_gyro.reset();
        }));

    m_operator_controller
        .rightTrigger(0.03)
        .whileTrue(
            Commands.run(
                () -> {
                  m_elevator.move(12 * m_operator_controller.getRightTriggerAxis());
                },
                m_elevator));

    m_operator_controller
        .leftTrigger(0.03)
        .whileTrue(
            Commands.run(
                () -> {
                  m_elevator.move(-12 * m_operator_controller.getLeftTriggerAxis());
                },
                m_elevator));

    m_swerve.setDefaultCommand(Commands.run(
        () -> {
          var fieldRelative = SmartDashboard.getBoolean("Use Field Relative", false);
          // the xbox controller we use has around a 5% center error
          var xDead = MathUtil.applyDeadband(m_driver_controller.getLeftY(), 0.05);
          var yDead = MathUtil.applyDeadband(m_driver_controller.getLeftX(), 0.05);
          var rotDead = MathUtil.applyDeadband(m_driver_controller.getRightX(), 0.05);
          // Get the x speed. We are inverting this because Xbox controllers return
          // negative values when we push forward.
          final var xSpeed = -xDead * Drivetrain.kMaxSpeed;

          // Get the y speed or sideways/strafe speed. We are inverting this because
          // we want a positive value when we pull to the left. Xbox controllers
          // return positive values when you pull to the right by default.
          final var ySpeed = -yDead * Drivetrain.kMaxSpeed;

          // Get the rate of angular rotation. We are inverting this because we want a
          // positive value when we pull to the left (remember, CCW is positive in
          // mathematics). Xbox controllers return positive values when you pull to
          // the right by default.
          final var rot = -rotDead * Drivetrain.kMaxAngularSpeed;

          m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
        }, m_swerve));

  }

}
