// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


public class SwerveModule {

  ///////////////////////////////////////////////////////////////
  // Update all these for your drivetrain
  
  // SDS Mk4i Modules
  private final double kWheelRadius_in = 3.0;
  private final double kWheelGearRatio = 6.75; 

  // The maximum speeds you _want_ your modules to spin. 
  // This should be at or below the theoretical maximum of the moduels you actually built.
  private final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  // Feedback and Feedforward constants
  private final double m_drive_kP = 0; 
  private final double m_drive_kI = 0.0;
  private final double m_drive_kD = 0;
  private final double m_drive_kV = 0;
  private final double m_drive_kS = 0.0;

  private final double m_turn_kP  = 5.0;
  private final double m_turn_kI  = 0.0;
  private final double m_turn_kD  = 0.01;
  private final double m_turn_kV  = 0.0; //TODO - we need to tune this
  private final double m_turn_kS  = 0.0; //TODO - we need to tune this


  // End you-update-em section
  ///////////////////////////////////////////////////////////////


  public final SparkMaxWrapper m_driveMotor;
  public final SparkMaxWrapper m_turningMotor;

  private final ThriftyEnocder m_turningEncoder;


  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          m_turn_kP,
          m_turn_kI,
          m_turn_kD,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(m_drive_kS, m_drive_kV);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(m_turn_kS, m_turn_kV);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param namePrefix String to uniquely identify this module
   * @param driveMotorChannel CAN ID for the drive motor.
   * @param turningMotorChannel CAN ID for the turning motor.
   * @param turningEncoderAnalogChannel Analog Input for the Thriftybot Analog Encoder
   */
  public SwerveModule(
      String namePrefix,
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderAnalogChannel,
      double turningEncoderOffset_rad) {

    // Set up drive NEO
    m_driveMotor = new SparkMaxWrapper(driveMotorChannel);
    m_driveMotor.setClosedLoopGains(m_drive_kP, m_drive_kI, m_drive_kD);

    // Set up turning NEO and the absolute encoder.
    m_turningMotor = new SparkMaxWrapper(turningMotorChannel);
    m_turningMotor.m_motor.setInverted(true);
    m_turningEncoder = new ThriftyEnocder(namePrefix + "_turning", turningEncoderAnalogChannel, turningEncoderOffset_rad);

    // Limit the turn PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    var tmp = dtMotorRotToLinear_m(m_driveMotor.getVelocity_radpersec());
    return new SwerveModuleState(tmp, m_turningEncoder.getPosition());
  }

  /**
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    var tmp = dtMotorRotToLinear_m(m_driveMotor.getPosition_rad());
    return new SwerveModulePosition(tmp, m_turningEncoder.getPosition());
  }

  /**
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, m_turningEncoder.getPosition());

    // Calculate the drive output with our own arbitrary feed-forward, 
    // but use the onboard PID control for the motor.
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    var velCmd_radPerSec = dtLinearToMotorRot_rad(state.speedMetersPerSecond);
    m_driveMotor.setClosedLoopCmd(velCmd_radPerSec, driveFeedforward);

    // Calculate the turning motor output from the turning PID controller.
    // Do this all onboard and just send a voltage command to the motor.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getPosition().getRadians(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_turningMotor.setVoltageCmd(turnOutput + turnFeedforward);
  }

  private double dtLinearToMotorRot_rad(double linear_m_in){
      return linear_m_in / (Units.inchesToMeters(kWheelRadius_in)) * kWheelGearRatio;
  }

  private double dtMotorRotToLinear_m(double motor_rad_in){
      return motor_rad_in * (Units.inchesToMeters(kWheelRadius_in)) / kWheelGearRatio;
  }

}
