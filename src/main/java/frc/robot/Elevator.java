package frc.robot;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    
    public final double m_kTOFbottom = 1; // inches
    public final double m_kTOFtop = 24; // inches
  
    public final SparkMaxWrapper m_motor = new SparkMaxWrapper(-1); // TODO get the CAN ID of the elevator's sparkmax
    public final TimeOfFlight m_tof = new TimeOfFlight(-1); // TODO get the sensor ID of the ToF sensor
    
    
    public Elevator() {
        m_tof.setRangingMode(RangingMode.Short, 25); // 40 times per second    
    }

    public boolean atBottom(){
        return getHeight() <= m_kTOFbottom;
    }

    public boolean atTop() {
        return getHeight() >= m_kTOFtop;
    }

    public void stop() {
        m_motor.setVoltageCmd(0);
    }

    public void move(double volts) {
        if (volts < 0 && atBottom()) {
            stop();
        } else if (volts > 0 && atTop()) {
            stop();
        } else {
            m_motor.setVoltageCmd(volts);
        }
        // if (m_controller.getLeftBumper() && rad < m_kTOFbottom) {
        //     m_motor.setClosedLoopCmd(m_kElevatorSpeed, m_kElevatorFeedForward);
        // } else if (m_controller.getRightBumper() && rad > m_kElevatorMinRadians) { 
        //     m_motor.setClosedLoopCmd(-m_kElevatorSpeed, m_kElevatorFeedForward);
        // }
    }

    public double getHeight() {
        // inches from bottom of elevator
        return m_tof.getRange() / 25.4;
    }

    public void robotPeriodic() {
        SmartDashboard.putNumber("tofRange", getHeight());

    }

}
