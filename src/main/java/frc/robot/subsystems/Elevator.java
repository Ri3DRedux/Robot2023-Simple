package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    
    public final double m_kTOFbottom = 1; // inches
    public final double m_kTOFtop = 24; // inches
  
    public final CANSparkMax m_motor = new CANSparkMax(10, MotorType.kBrushless);
    public final TimeOfFlight m_tof = new TimeOfFlight(1);
    public final RelativeEncoder m_encoder;

    public Elevator() {
        m_tof.setRangingMode(RangingMode.Short, 25); // 40 times per second

        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setSmartCurrentLimit(40);
        m_motor.setInverted(true);

        m_encoder = m_motor.getEncoder();
        m_encoder.setPositionConversionFactor(1/3.47);

        stop();


        setDefaultCommand(Commands.runOnce(() -> { stop(); }, this));
    }

    public boolean atBottom(){
        return getHeight() <= m_kTOFbottom;
    }

    public boolean atTop() {
        return getHeight() >= m_kTOFtop;
    }

    public void stop() {
        m_motor.setVoltage(0);
    }

    public void move(double volts) {
        if (volts < 0 && atBottom()) {
            stop();
        } else if (volts > 0 && atTop()) {
            stop();
        } else {
            m_motor.setVoltage(volts);
        }
    }

    public double getToFHeight() {
        // inches from bottom of elevator
        return m_tof.getRange() / 25.4;
    }

    public double getMotorHeight() {
        return m_encoder.getPosition();
    }

    public double getHeight() {
        return getMotorHeight();
    }

    public void periodic() {
        SmartDashboard.putNumber("elevator tofHeight", getToFHeight());
        SmartDashboard.putNumber("elevator motorHeight", getMotorHeight());

        var currVoltage = m_motor.get() * 12;
        if (currVoltage < 0 && atBottom()) {
            stop();
        } else if (currVoltage > 0 && atTop()) {
            stop();
        }
    }
}
