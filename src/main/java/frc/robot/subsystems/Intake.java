package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake extends SubsystemBase {

    public final CANSparkMax m_motor_left = new CANSparkMax(-1, MotorType.kBrushed); // TODO get the CAN ID of the intake's left sparkmax
    public final CANSparkMax m_motor_right = new CANSparkMax(-1, MotorType.kBrushed); // TODO get the CAN ID of the intake's right sparkmax
    public final TimeOfFlight m_tof = new TimeOfFlight(-1); // TODO get a CAN ID for the ToF sensor

    // TODO use the ToF to run intake until it detects an object was captured

    public Intake() {

        m_motor_left.restoreFactoryDefaults();
        m_motor_left.setIdleMode(IdleMode.kBrake);
        m_motor_left.setSmartCurrentLimit(40);
        m_motor_left.burnFlash();

        m_motor_right.restoreFactoryDefaults();
        m_motor_right.setIdleMode(IdleMode.kBrake);
        m_motor_right.setSmartCurrentLimit(40);
        m_motor_right.burnFlash();

        stop();
        setDefaultCommand(Commands.runOnce(() -> { stop(); }, this));

        m_tof.setRangingMode(RangingMode.Short, 25); // 40 times per second
    }

    public void stop() {
        m_motor_left.setVoltage(0);
        m_motor_right.setVoltage(0);
    }

    public void periodic() {
        // TODO
    }

    public void moveLeftMotor(double volts) {
        m_motor_left.setVoltage(volts);
    }

    public void moveRightMotor(double volts) {
        m_motor_right.setVoltage(volts);
    }

    public void intake() {
        intake(0, 0);
    }

    public void intake(double volts) {
        m_motor_left.setVoltage(-volts);
        m_motor_right.setVoltage(-volts);
    }

    public void intake(double leftMotorVolts, double rightMotorVolts) {
        m_motor_left.setVoltage(-leftMotorVolts);
        m_motor_right.setVoltage(-rightMotorVolts);
    }

    public void outtake() {
        outtake(0,0);
    }

    public void outtake(double volts) {
        m_motor_left.setVoltage(volts);
        m_motor_right.setVoltage(volts);
    }

    public void outtake(double leftMotorVolts, double rightMotorVolts) {
        m_motor_left.setVoltage(leftMotorVolts);
        m_motor_right.setVoltage(rightMotorVolts);

    }

    public boolean havePossession() {
        return m_tof.getRange() < 40;
    }

    public CommandBase intakeCmd() {
        return this.run(() -> {
            intake();
        }).until(this::havePossession);
    }

}
