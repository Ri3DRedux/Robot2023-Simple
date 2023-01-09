// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.camera.PhotonCamWrapper;
import frc.robot.patch.SwerveDrivePoseEstimator;


/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

  ///////////////////////////////////////////////////////////////
  // Update all these for your drivetrain

  // The maximum speed you _want_ the drivetrain to go. It should be at or below
  // the theoretical maximum speeds of the drivetrain you actually built.
  public static final double kMaxSpeed = 2.0; // 2 meters per second
  public static final double kMaxAngularSpeed = Math.PI*2; // 2 rotation per second

  // Physical dimensions of the drivetrain. What's important here is the
  // width and length between the contact patches of the wheels touching the
  // ground
  // (which likely isn't the same as your frame perimiter)
  private final double trackWidth_m = Units.inchesToMeters(19.5);
  private final double trackLength_m = Units.inchesToMeters(25);

  // Mechanical mounting offsets of the encoder & magnet within the shaft
  // Must be updated whenever the module is reassembled
  // Procedure:
  // 0 - Put the robot up on blocks.
  // 1 - Reset all these values to 0, deploy code
  // 2 - Pull up dashboard with encoder readings (in radians)
  // 3 - Using a square, twist the modules by hand until they are aligned with the
  // robot's chassis
  // 4 - Read out the encoder readings for each module, put them here
  // 5 - Redeploy code, verify that hte encoder readings are correct as each
  // module is manually rotated
  private final double FL_ENCODER_MOUNT_OFFSET_RAD = -Units.degreesToRadians(-137.0);
  private final double FR_ENCODER_MOUNT_OFFSET_RAD = -Units.degreesToRadians(-155.0);
  private final double BL_ENCODER_MOUNT_OFFSET_RAD = -Units.degreesToRadians(147.0);
  private final double BR_ENCODER_MOUNT_OFFSET_RAD = -Units.degreesToRadians(134.0);

  // End you-update-em section
  ///////////////////////////////////////////////////////////////

  private final Translation2d m_frontLeftLocation = new Translation2d(trackWidth_m / 2.0, trackLength_m / 2.0);
  private final Translation2d m_frontRightLocation = new Translation2d(trackWidth_m / 2.0, -trackLength_m / 2.0);
  private final Translation2d m_backLeftLocation = new Translation2d(-trackWidth_m / 2.0, trackLength_m / 2.0);
  private final Translation2d m_backRightLocation = new Translation2d(-trackWidth_m / 2.0, -trackLength_m / 2.0);

  public final SwerveModule m_frontLeft = new SwerveModule("FL", 1, 2, 0, FL_ENCODER_MOUNT_OFFSET_RAD);
  public final SwerveModule m_frontRight = new SwerveModule("FR", 3, 4, 1, FR_ENCODER_MOUNT_OFFSET_RAD);
  public final SwerveModule m_backLeft = new SwerveModule("BL", 5, 6, 2, BL_ENCODER_MOUNT_OFFSET_RAD);
  public final SwerveModule m_backRight = new SwerveModule("BR", 7, 8, 3, BR_ENCODER_MOUNT_OFFSET_RAD);

  ///////////////////////////////////////////////////////////////
  // Variables for Auto Alignment Based on Pose Estimation

  // Relative Field coordinates for April Tag ID #1; taken from game manual
  private static final double APRIL_TAG_1_ABSOLUTE_X_COORD_INCHES = 610.77;
  private static final double APRIL_TAG_1_ABSOLUTE_Y_COORD_INCHES = 42.19;
  private static final double APRIL_TAG_1_ABSOLUTE_ROT_DEGREES    = 180.0;

  private static final Pose2d april_tag_1_pose2d = new Pose2d(APRIL_TAG_1_ABSOLUTE_X_COORD_INCHES, APRIL_TAG_1_ABSOLUTE_Y_COORD_INCHES, new Rotation2d(APRIL_TAG_1_ABSOLUTE_ROT_DEGREES));

  private static final double AUTO_ALIGN_X_DISTANCE_FROM_TAG_INCHES = 32.0;

  private static final double AUTO_ALIGN_X_ERROR_TOLERANCE_IN    = 0.5;
  private static final double AUTO_ALIGN_Y_ERROR_TOLERANCE_IN    = 0.5;
  private static final double AUTO_ALIGN_ROT_ERROR_TOLERANCE_DEG = 1.0;

  private static final double autoAlignX_kP = 1.0;
  private static final double autoAlignX_kI = 0.0;
  private static final double autoAlignX_kD = 0.0;

  private static final double autoAlignY_kP = 1.0;
  private static final double autoAlignY_kI = 0.0;
  private static final double autoAlignY_kD = 0.0;

  private static final double autoAlignRot_kP = 1.0;
  private static final double autoAlignRot_kI = 0.0;
  private static final double autoAlignRot_kD = 0.0;
  
  private PIDController autoAlignX_PIDController   = new PIDController(autoAlignX_kP, autoAlignX_kI, autoAlignX_kD);
  private PIDController autoAlignY_PIDController   = new PIDController(autoAlignY_kP, autoAlignY_kI, autoAlignY_kD);
  private PIDController autoAlignRot_PIDController = new PIDController(autoAlignRot_kP, autoAlignRot_kI, autoAlignRot_kD);

  ///////////////////////////////////////////////////////////////
  
  // Camera auto-align to target things
  public PhotonCamWrapper cam_front = new PhotonCamWrapper("White_Cam", new Transform3d(new Translation3d(trackLength_m/2.0, 0, 0), new Rotation3d(0.0, 0.0, 0.0)));
  public PhotonCamWrapper cam_left = new PhotonCamWrapper("Red_Cam", new Transform3d(new Translation3d(0, trackWidth_m/2.0, 0), new Rotation3d(0.0, 0.0, Math.PI/2.0)));
  public ArrayList<PhotonCamWrapper> camSet = new ArrayList<>(List.of(cam_front, cam_left));

  public final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // private final PhotonCamWrapper m_cam = new PhotonCamWrapper(photonCamName, photonCamMountLocation);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private Field2d field = new Field2d();

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used
   * below are robot specific, and should be tuned.
   */
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      },
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  /**
   * Constructor
   */
  public Drivetrain() 
  {
    m_gyro.reset();
    SmartDashboard.putData("Field", field);

    autoAlignX_PIDController.setTolerance(AUTO_ALIGN_X_ERROR_TOLERANCE_IN);
    autoAlignY_PIDController.setTolerance(AUTO_ALIGN_Y_ERROR_TOLERANCE_IN);
    autoAlignRot_PIDController.setTolerance(AUTO_ALIGN_ROT_ERROR_TOLERANCE_DEG);
  }

  public CommandBase getAutoAlignCommand()
  {
    return new CommandBase()
    {
      private Pose2d robot_pose;
      private Pose2d april_tag_1_pose;

      private Pose2d desired_robot_pose;

      private void getPoseUpdates()
      {
        robot_pose = m_poseEstimator.getEstimatedPosition();
        desired_robot_pose = new Pose2d((april_tag_1_pose.getX() - AUTO_ALIGN_X_DISTANCE_FROM_TAG_INCHES), (april_tag_1_pose.getY()), new Rotation2d(0.0));
      }

      public void initialize()
      {
        // runs once when the command starts

        april_tag_1_pose = april_tag_1_pose2d;

        getPoseUpdates();
      }

      public void execute()
      {
        getPoseUpdates();

        double new_x_command   = autoAlignX_PIDController.calculate(robot_pose.getX(), desired_robot_pose.getX());
        double new_y_command   = autoAlignY_PIDController.calculate(robot_pose.getY(), desired_robot_pose.getY());
        double new_rot_command = autoAlignRot_PIDController.calculate(robot_pose.getRotation().getDegrees(), desired_robot_pose.getRotation().getDegrees());

        drive(new_x_command, new_y_command, new_rot_command, false);

        // loops until isFinished() == true
      }

      public boolean isFinished()
      {
        boolean at_x, at_y, at_rot;
        at_x   = autoAlignX_PIDController.atSetpoint();
        at_y   = autoAlignY_PIDController.atSetpoint();
        at_rot = autoAlignRot_PIDController.atSetpoint();

        // end condition
        return (at_x && at_y && at_rot);
      }
    };
  }

  public void resetWheelsToForward(){
    m_frontLeft.m_turningMotor.m_encoder.setPosition(0.0);
    m_backLeft.m_turningMotor.m_encoder.setPosition(0.0);
    m_frontRight.m_turningMotor.m_encoder.setPosition(0.0);
    m_backRight.m_turningMotor.m_encoder.setPosition(0.0);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void periodic(){
    updateOdometry();
  }

  // Updates the field relative position of the robot.
  public void updateOdometry() {

    m_frontLeft.readAngleSensors();
    m_frontRight.readAngleSensors();
    m_backLeft.readAngleSensors();
    m_backRight.readAngleSensors();

    SmartDashboard.putNumber("gyro_angle", m_gyro.getRotation2d().getDegrees());
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });

    // Put all vision observations into the pose estimator
    for(var cam : camSet){
      cam.update();
      for (var obs : cam.getCurObservations()) {
        m_poseEstimator.addVisionMeasurement(obs.estFieldPose, obs.time);
      }
    }

    field.getObject("Robot").setPose(m_poseEstimator.getEstimatedPosition());
  }

  public void setKnownPosition(Pose2d pos) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }, pos);

  }
}
