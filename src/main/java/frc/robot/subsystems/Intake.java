package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake extends SubsystemBase {
    public final TalonSRX m_motor_left = new TalonSRX(0);
    public final TalonSRX m_motor_right = new TalonSRX(8); 
    public final TimeOfFlight m_tof = new TimeOfFlight(0); 

    // TODO use the ToF to run intake until it detects an object was captured

    public Intake() {
        m_motor_left.setNeutralMode(NeutralMode.Brake);
        //m_motor_left.restoreFactoryDefaults();
        //m_motor_left.setIdleMode(IdleMode.kBrake);
        //m_motor_left.setSmartCurrentLimit(40);
        //m_motor_left.burnFlash();

        m_motor_right.setNeutralMode(NeutralMode.Brake);
        m_motor_right.setInverted(true);
        //m_motor_right.restoreFactoryDefaults();
        //m_motor_right.setIdleMode(IdleMode.kBrake);
        //m_motor_right.setSmartCurrentLimit(40);
        //m_motor_right.burnFlash();

        stop();
        setDefaultCommand(Commands.runOnce(() -> { stop(); }, this));

        m_tof.setRangingMode(RangingMode.Short, 25); // 40 times per second
        m_tof.setRangeOfInterest(8, 8, 12, 12);
    }

    public void stop() {
        m_motor_left.set(ControlMode.PercentOutput,0);
        m_motor_right.set(ControlMode.PercentOutput,0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake TOF", m_tof.getRange());
    }

    public void moveLeftMotor(double volts) {
        m_motor_left.set(ControlMode.PercentOutput,volts/RobotController.getBatteryVoltage());
    }

    public void moveRightMotor(double volts) {
        m_motor_right.set(ControlMode.PercentOutput,volts/RobotController.getBatteryVoltage());
    }

    public void intake() {
        intake(0, 0);
    }

    public void intake(double volts) {
        m_motor_left.set(ControlMode.PercentOutput,-volts/RobotController.getBatteryVoltage());
        m_motor_right.set(ControlMode.PercentOutput,-volts/RobotController.getBatteryVoltage());
    }

    public void intake(double leftMotorVolts, double rightMotorVolts) {
        m_motor_left.set(ControlMode.PercentOutput,-leftMotorVolts/RobotController.getBatteryVoltage());
        m_motor_right.set(ControlMode.PercentOutput,-rightMotorVolts/RobotController.getBatteryVoltage());
    }

    public void outtake() {
        outtake(0,0);
    }

    public void outtake(double volts) {
        m_motor_left.set(ControlMode.PercentOutput,volts/RobotController.getBatteryVoltage());
        m_motor_right.set(ControlMode.PercentOutput,volts/RobotController.getBatteryVoltage());
    }

    public void outtake(double leftMotorVolts, double rightMotorVolts) {
        m_motor_left.set(ControlMode.PercentOutput,leftMotorVolts/RobotController.getBatteryVoltage());
        m_motor_right.set(ControlMode.PercentOutput,rightMotorVolts/RobotController.getBatteryVoltage());

    }

    public boolean havePossession() {
        return m_tof.getRange() < 140;
    }

    public CommandBase intakeCmd() {
        return this.run(() -> {
            intake();
        }).until(this::havePossession);
    }

}
