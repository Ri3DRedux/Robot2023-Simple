package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Small class to wrapper a Thriftybot analog encoder
 */
public class ThriftyEnocder {

    private final String name;

    private double magnetOffset_rad;

    // Keep separate analog input and encoder classes
    // to get finer-grained debug information
    private final AnalogInput m_input;
    private final AnalogEncoder m_enc;

    public ThriftyEnocder(String namePrefix, int port, double magnetOffset_rad) {
        m_input = new AnalogInput(port);
        m_enc = new AnalogEncoder(m_input);
        name = namePrefix + "_TBEnc";
        this.magnetOffset_rad = magnetOffset_rad;

    }

    public Rotation2d getPosition() {
        // getAbsolutePosition returns a number from 0 to 1 which
        // corresponds to 0-2PI Radians. We do that conversion and send it to
        // a Rotation2d.
        // See
        // https://www.thethriftybot.com/bearings/Thrifty-Absolute-Magnetic-Encoder-p421607500
        // for more hardware interfacing information.
        var encoderReadingRaw = m_enc.getAbsolutePosition();
        var retVal = new Rotation2d(encoderReadingRaw * Math.PI * 2 + this.magnetOffset_rad);

        SmartDashboard.putNumber(name + "_raw_voltage", m_input.getVoltage());
        SmartDashboard.putNumber(name + "_pos_deg", retVal.getDegrees());

        return retVal;
    }

}
