package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public final CANSparkMax m_motor_left = new CANSparkMax(-1, MotorType.kBrushed); // TODO get the CAN ID of the intake's left sparkmax
    public final CANSparkMax m_motor_right = new CANSparkMax(-1, MotorType.kBrushed); // TODO get the CAN ID of the intake's right sparkmax

    public Intake() {
        stop();
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
}
