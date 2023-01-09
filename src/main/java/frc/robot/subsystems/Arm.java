package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    public CANSparkMax m_motor = new CANSparkMax(-1, MotorType.kBrushed);
    public RelativeEncoder m_encoder;
    public final double SCALE_FACTOR = 1;

    public Arm() {
        m_encoder = m_motor.getEncoder();
        m_encoder.setPositionConversionFactor(SCALE_FACTOR);
    }

    public void periodic() {
        SmartDashboard.putNumber("Arm Position", m_encoder.getPosition());
    }
    
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

}
