// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pincher;

public class Robot extends TimedRobot {
  private final CommandXboxController m_driver_controller = new CommandXboxController(0);
  private final CommandXboxController m_operator_controller = new CommandXboxController(1);
  public final Drivetrain swerve = new Drivetrain();
  public final Arm arm = new Arm(this);
  public final Intake intake = new Intake(this);
  public final Pincher pincher = new Pincher(this);

  public BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  private final Pose2d m_autoStartPose = new Pose2d();
  private final PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  private final PneumaticsControlModule pcm = new PneumaticsControlModule();
  private NetworkTable table;

  Vision vision = new Vision();

  @Override
  public void robotInit() {
    // Starts recording to data log
    DataLogManager.start();
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
    table = NetworkTableInstance.getDefault().getTable("/status");

    CommandScheduler.getInstance().registerSubsystem(swerve, arm, intake, pincher);

    configureButtonBindings();

    SmartDashboard.putBoolean("Use Field Relative", false);
    // new LoopTimeLogger(this);
    // cam.setVersionCheckEnabled(false);

    // vision.start();
    pcm.enableCompressorDigital();

    DriverStation.silenceJoystickConnectionWarning(true);

  }

  @Override
  public void disabledInit() {
    pincher.pinch();
    intake.down();
  }

  @Override
  public void disabledPeriodic() {
    if (RobotController.getUserButton()) {
      swerve.resetWheelsToForward();
    }

  }

  @Override
  public void autonomousInit() {
    swerve.setKnownPosition(m_autoStartPose);

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

    SmartDashboard.putNumber("ramp/accel x", accelerometer.getX());
    SmartDashboard.putNumber("ramp/accel y", accelerometer.getY());
    SmartDashboard.putNumber("ramp/accel z", accelerometer.getZ());

    SmartDashboard.putNumber("ramp/gyro height", swerve.m_gyro.getDisplacementZ());
  }

  public void configureButtonBindings() {
    m_driver_controller.a().onTrue(
        Commands.runOnce(() -> {
          swerve.m_gyro.reset();
        }));

    swerve.setDefaultCommand(Commands.run(
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

          swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
        }, swerve));

    m_driver_controller.leftBumper().whileTrue(Commands.runOnce(() -> {
      var xDead = accelerometer.getX() * 2;

      var yDead = 0;

      final var xSpeed = -xDead * Drivetrain.kMaxSpeed;

      final var ySpeed = -yDead * Drivetrain.kMaxSpeed;

      var rot = 0;

      swerve.drive(xSpeed, ySpeed, rot, false);

    }, swerve)
    .andThen(
      new WaitCommand(0.2)
    ).andThen(
      Commands.runOnce(() -> {
      swerve.drive(0, 0, 0, false);
    }, swerve).andThen(new WaitCommand(1))
      ).repeatedly()
    );

    class ArmDefaultCommand extends CommandBase {

      double lastControlleredPosition;

      public ArmDefaultCommand() {
        addRequirements(arm);
      }

      public void initialize() {
        lastControlleredPosition = arm.getPivotPosition();
      }

      public void execute() {
        var leftY = m_operator_controller.getLeftY();
        var rightY = m_operator_controller.getRightY();

        // leftY = MathUtil.applyDeadband(leftY, 0.03);

        if (Math.abs(leftY) > 0.03) {
          arm.movePivotUnsafe(-12 * leftY);
          lastControlleredPosition = arm.getPivotPosition();
          arm.pivotPIDController.reset();
        } else {
          arm.movePivotUnsafe(-12 * leftY);
          // return;

          var pivotFeedbackV = arm.pivotPIDController.calculate(arm.getPivotPosition(), lastControlleredPosition);
          // Feedforward - delete gravity
          var pivotFeedforwardV = Math.cos(Units.degreesToRadians(arm.getPivotPosition())) * arm.pivot_kG;

          var volts = pivotFeedbackV + pivotFeedforwardV;

          // volts = MathUtil.clamp(volts, -4, 4);

          System.out
              .println(String.valueOf(-12 * leftY).substring(0, 4) + ", " + String.valueOf(volts).substring(0, 4));
          arm.movePivotUnsafe(pivotFeedforwardV);
        }

        arm.moveExtensionUnsafe(-12 * m_operator_controller.getRightY());
      }
    }

    arm.setDefaultCommand(new ArmDefaultCommand());

    // arm.setDefaultCommand(Commands.run(
    // () -> {

    // var leftY = m_operator_controller.getLeftY();
    // var rightY = m_operator_controller.getRightY();

    // leftY = MathUtil.applyDeadband(leftY, 0.03);

    // if (leftY > 0.03) {
    // arm.movePivotUnsafe(-12 * leftY);
    // } else {
    // double volts =
    // arm.extensionPIDController.calculate(arm.getExtensionPosition(), inches);
    // volts = MathUtil.clamp(volts, -EXTENSION_MAX_VOLTS, EXTENSION_MAX_VOLTS);
    // }

    // arm.moveExtensionUnsafe(-12 * m_operator_controller.getRightY());

    // }, arm));

    m_operator_controller.leftBumper().onTrue(pincher.releaseCmd());
    m_operator_controller.rightBumper().onTrue(pincher.pinchCmd());

    m_operator_controller.a().onTrue(intake.downCmd());
    m_operator_controller.b().onTrue(intake.upCmd());

    m_operator_controller.x().whileTrue(
        Commands.run(() -> {
          intake.intake();
        }, intake));

    m_operator_controller.y().whileTrue(
        Commands.run(() -> {
          intake.outtake();
        }, intake));

  }

  // TODO: Fill in channel names with actual function names
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
    for (int i = 0; i < pdpChannelNames.length; i++) {
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
