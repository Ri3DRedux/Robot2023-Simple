// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class Robot extends TimedRobot {
  private final CommandXboxController m_driver_controller = new CommandXboxController(0);
  private final CommandXboxController m_operator_controller = new CommandXboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();

  private final Pose2d m_autoStartPose = new Pose2d();

  Vision vision = new Vision();

  @Override
  public void robotInit() {
    // Starts recording to data log
    DataLogManager.start();
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    CommandScheduler.getInstance().registerSubsystem(m_swerve, m_arm, m_intake);

    configureButtonBindings();

    SmartDashboard.putBoolean("Use Field Relative", false);

    // cam.setVersionCheckEnabled(false);

    vision.start();

  }

  @Override
  public void disabledPeriodic() {
    if (RobotController.getUserButton()) {
      m_swerve.resetWheelsToForward();
    }

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

    m_driver_controller.rightBumper().onTrue(Commands.run(() -> {

      // the xbox controller we use has around a 5% center error
      var xDead = MathUtil.applyDeadband(m_driver_controller.getLeftY(), 0.05);
      var yDead = MathUtil.applyDeadband(m_driver_controller.getLeftX(), 0.05);
      // Get the x speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      final var xSpeed = -xDead * Drivetrain.kMaxSpeed;

      // Get the y speed or sideways/strafe speed. We are inverting this because
      // we want a positive value when we pull to the left. Xbox controllers
      // return positive values when you pull to the right by default.
      final var ySpeed = -yDead * Drivetrain.kMaxSpeed;

      var rot = 0;

      // if (targetVisible) {
      //   var angleErr = camAngle - CAM_ALIGN_TARGET_YAW;
      //   rot = angleErr * CAM_ALIGN_P_GAIN;
      // } 

      m_swerve.drive(xSpeed, ySpeed, rot, false);

    }, m_swerve));
  }

  public void loggingPeriodic() {
    PowerDistribution pdp = new PowerDistribution();
    var channels = pdp.getNumChannels();  //ask once, use many times
    for(int i=0; i<channels; i++) {
      SmartDashboard.putNumber("PDP Current " + i, pdp.getCurrent(i));
    }
    SmartDashboard.putNumber("PDP Voltage", pdp.getVoltage());
    SmartDashboard.putNumber("PDP Total Current", pdp.getTotalCurrent());

    var canStatus = RobotController.getCANStatus();
    SmartDashboard.putNumber("CAN Bandwidth", canStatus.percentBusUtilization);
    //canStatus.busOffCount
    canStatus.
  }
}
