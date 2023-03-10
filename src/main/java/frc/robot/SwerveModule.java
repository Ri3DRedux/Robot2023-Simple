// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveModule {

  ///////////////////////////////////////////////////////////////
  // Update all these for your drivetrain
  
  // SDS Mk4i Modules
  private final double kWheelRadius_in = 3.0;
  private final double kWheelGearRatio = 6.75; 

  // Feedback and Feedforward constants
  private final double m_drive_kP = 0; 
  private final double m_drive_kI = 0.0;
  private final double m_drive_kD = 0;
  private final double m_drive_kV = 12.0 / 4.4196; //Volts per max m/sec
  private final double m_drive_kS = 0.0;

  private final double m_turn_kP  = 7.0;
  private final double m_turn_kI  = 0.01;
  private final double m_turn_kD  = 0.0;


  // End you-update-em section
  ///////////////////////////////////////////////////////////////


  public final SparkMaxWrapper m_driveMotor;
  public final SparkMaxWrapper m_turningMotor;

  private final ThriftyEncoder m_turningEncoder;

  private String swerveName;

  private Rotation2d tbangle;
  private Rotation2d motorAngle;


  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_turningPIDController =
      new PIDController(
          m_turn_kP,
          m_turn_kI,
          m_turn_kD);

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(m_drive_kS, m_drive_kV);

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
    m_turningEncoder = new ThriftyEncoder(namePrefix + "_turning", turningEncoderAnalogChannel, turningEncoderOffset_rad);

    // Limit the turn PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    swerveName = namePrefix;
  }

  public void readAngleSensors(){
    motorAngle = new Rotation2d((m_turningMotor.getPosition_rad()) / (150/7));
    tbangle = m_turningEncoder.getPosition();
    var tbangle_deg = Units.radiansToDegrees(MathUtil.angleModulus(m_turningEncoder.getPosition().getRadians()));
    var motorAngle_deg = Units.radiansToDegrees(MathUtil.angleModulus(motorAngle.getRadians()));
    SmartDashboard.putNumber(swerveName + " ThriftyEnc Angle", tbangle_deg);
    SmartDashboard.putNumber(swerveName + " Motor Angle", motorAngle_deg);
  }

  /**
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {

    var tmp = dtMotorRotToLinear_m(m_driveMotor.getPosition_rad());
    return new SwerveModulePosition(tmp, motorAngle);
  }

  /**
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, motorAngle);

    // Calculate the drive output with our own arbitrary feed-forward, 
    // but use the onboard PID control for the motor.
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    var velCmd_radPerSec = dtLinearToMotorRot_rad(state.speedMetersPerSecond);
    m_driveMotor.setClosedLoopCmd(velCmd_radPerSec, driveFeedforward);

    // Calculate the turning motor output from the turning PID controller.
    // Do this all onboard and just send a voltage command to the motor.
    final double turnOutput =
        m_turningPIDController.calculate(motorAngle.getRadians(), state.angle.getRadians());

    SmartDashboard.putNumber(swerveName + " Des Angle", state.angle.getDegrees());

    SmartDashboard.putNumber(swerveName + " V", turnOutput);


    m_turningMotor.setVoltageCmd(turnOutput);
  }

  private double dtLinearToMotorRot_rad(double linear_m_in){
      return linear_m_in / (Units.inchesToMeters(kWheelRadius_in)) * kWheelGearRatio;
  }

  private double dtMotorRotToLinear_m(double motor_rad_in){
      return motor_rad_in * (Units.inchesToMeters(kWheelRadius_in)) / kWheelGearRatio;
  }

}
