// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.camera.PhotonCamWrapper;

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
  private final double trackWidth_m = Units.inchesToMeters(27);
  private final double trackLength_m = Units.inchesToMeters(27);

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

  
  // Camera auto-align to target things
  public final PhotonCamWrapper cam = new PhotonCamWrapper("Black_Cam");
  public static final double CAM_ALIGN_P_GAIN = Drivetrain.kMaxAngularSpeed * 0.2 / 40.0;// radpersec per degree
  public static final double CAM_ALIGN_TARGET_YAW = 0.0;


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

  public Drivetrain() {
    m_gyro.reset();
    SmartDashboard.putData("Field", field);
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
    updateCamera();
    updateOdometry();
  }

  public void updateCamera(){
    //Vision closed-loop alignment - if requested, override 
    // rotation by a closed-loop P control to turn toward the target
    var camAngle = cam.getTgtYaw();


    SmartDashboard.putBoolean("Cam Target Visible", camAngle.isPresent());
    SmartDashboard.putNumber("Cam Target Angle Deg", camAngle.orElse(0.0));
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
