package frc.robot;

import com.playingwithfusion.TimeOfFlight;

public class Elevator {
    
    private final double m_kTOFbottom = 0; // TODO calculate the radians using the motor's internal encoder at the max height of the elevator (and reduce it a little as a safety margin)
    private final double m_kTOPtop = 0;   // TODO about 5% of the max radians (as a safety margin)
  
    private final SparkMaxWrapper m_motor = new SparkMaxWrapper(-1); // TODO get the CAN ID of the elevator's sparkmax
    // private final TimeOfFlight 

    private void move(double speed) {
        // double rad = m_motor.getPosition_rad();
        // if (m_controller.getLeftBumper() && rad < m_kTOFbottom) {
        //     m_motor.setClosedLoopCmd(m_kElevatorSpeed, m_kElevatorFeedForward);
        // } else if (m_controller.getRightBumper() && rad > m_kElevatorMinRadians) { 
        //     m_motor.setClosedLoopCmd(-m_kElevatorSpeed, m_kElevatorFeedForward);
        // }
      }

}
