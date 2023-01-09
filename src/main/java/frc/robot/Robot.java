// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class Robot extends TimedRobot {
  private final CommandXboxController m_driver_controller = new CommandXboxController(0);
  private final CommandXboxController m_operator_controller = new CommandXboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();
  private final Elevator m_elevator = new Elevator();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  private final Pose2d m_autoStartPose = new Pose2d();
  private final PowerDistribution pdp = new PowerDistribution(0,ModuleType.kCTRE);
  private NetworkTable table;

  @Override
  public void robotInit() {
    // Starts recording to data log
    DataLogManager.start();
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
    table = NetworkTableInstance.getDefault().getTable("/status");

    CommandScheduler.getInstance().registerSubsystem(m_elevator, m_swerve, m_arm, m_intake);
    configureButtonBindings();

    SmartDashboard.putBoolean("Use Field Relative", false);
    new LoopTimeLogger(this);
    //cam.setVersionCheckEnabled(false);
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
    loggingPeriodic();
  }

  public void configureButtonBindings() {

    m_driver_controller.a().onTrue(
        Commands.runOnce(() -> {
          m_swerve.m_gyro.reset();
        }));

    m_operator_controller // elevator up
        .rightTrigger(0.03)
        .whileTrue(
            Commands.run(
                () -> {
                  m_elevator.move(12 * m_operator_controller.getRightTriggerAxis());
                },
                m_elevator));

    m_operator_controller // elevator down
        .leftTrigger(0.03)
        .whileTrue(
            Commands.run(
                () -> {
                  m_elevator.move(-12 * m_operator_controller.getLeftTriggerAxis());
                },
                m_elevator));

    m_operator_controller // intake
        .leftBumper()
        .whileTrue(
            Commands.run(() -> {
              m_intake.intake(12);
            }, m_intake));

    m_operator_controller // outtake
        .rightBumper()
        .whileTrue(
            Commands.run(() -> {
              m_intake.outtake(12);
            }, m_intake));

    m_arm.setDefaultCommand(Commands.run(
        () -> {
          var voltage = m_operator_controller.getRightY() * 3; // should we put a deadband on this?
          m_arm.move(voltage);
        },m_arm));

    m_swerve.setDefaultCommand(Commands
      .run(
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

      var rot = 0.0;

      //@Vasista inject the wpi apriltag stuff here 
      var camYaw = m_swerve.cam.getTgtYaw();

      if (camYaw.isPresent()) {
        var angleErr = camYaw.get() - Drivetrain.CAM_ALIGN_TARGET_YAW;
        rot = angleErr * Drivetrain.CAM_ALIGN_P_GAIN;
      } 

      m_swerve.drive(xSpeed, ySpeed, rot, false);

    }, m_swerve));

  }

  //TODO: Fill in channel names with actual function names
  public String[] pdpChannelNames = {
    "0",
    "1",
    "2",
    "3",
    "4",
    "5",
    "6",
    "7",
    "8",
    "9",
    "10",
    "11",
    "12",
    "13",
    "14",
    "15"
  };

  public void loggingPeriodic() {
    for(int i=0; i<pdpChannelNames.length; i++) {
      table.getEntry("PDP Current " + pdpChannelNames[i]).setDouble(pdp.getCurrent(i));
    }
    table.getEntry("PDP Voltage").setDouble(pdp.getVoltage());
    table.getEntry("PDP Total Current").setDouble(pdp.getTotalCurrent());
  
    var canStatus = RobotController.getCANStatus();
    table.getEntry("CAN Bandwidth").setDouble(canStatus.percentBusUtilization);
    table.getEntry("CAN Bus Off Count").setDouble(canStatus.busOffCount);
    table.getEntry("CAN RX Error Count").setDouble(canStatus.receiveErrorCount);
    table.getEntry("CAN Tx Error Count").setDouble(canStatus.transmitErrorCount);
    table.getEntry("CAN Tx Full Count").setDouble(canStatus.txFullCount);

    table.getEntry("Rio 3.3V Voltage").setDouble(RobotController.getVoltage3V3());
    table.getEntry("Rio 5V Voltage").setDouble(RobotController.getVoltage5V());
    table.getEntry("Rio 6V Voltage").setDouble(RobotController.getVoltage6V());
    table.getEntry("Rio 3.3V Current").setDouble(RobotController.getCurrent3V3());
    table.getEntry("Rio 5V Current").setDouble(RobotController.getCurrent5V());
    table.getEntry("Rio 6V Current").setDouble(RobotController.getCurrent6V());
  }
}
