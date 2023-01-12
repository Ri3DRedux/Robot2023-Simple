package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivotSensor {
    
    // DutyCycleEncoder pivotEncoder;
    DutyCycle pivotEncoder;
    // DutyCycleencoder should have a way to set range/offset by taking in any 2 points instead of the min/max
    // Example API
    // dutycycleencoder.setPositionConversion((rawSensorValue1, expectedOutput1), (rawSensorValue2, expectedOutput2));

    //TODO TWEAK THESE FOR WHAT THE REAL ROBOT IS
    public final double MIN_DUTY_CYCLE = 0.998;
    public final double MAX_DUTY_CYCLE = 0.167;
    public final double MIN_ANGLE = -55.7;
    public final double MAX_ANGLE = 180+65.3;

    public final double REAL_MIN_ANGLE = -53;
    public final double REAL_MAX_ANGLE = 230;

    public PivotSensor(int id) {
        // pivotEncoder = new DutyCycleEncoder(id);
        pivotEncoder = new DutyCycle(new DigitalInput(id));
    }

    public double getAngleDeg(){
        // var rawDC = pivotEncoder.getAbsolutePosition()
        var rawDC = pivotEncoder.getOutput();
        //Frac is 0 at lowest point, 1 at max extension
        var dcFrac = (rawDC - MIN_DUTY_CYCLE) / (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);
        var angle = MIN_ANGLE + dcFrac * (MAX_ANGLE - MIN_ANGLE);

        if (angle > 280) {
            angle -= 360;
        }

        SmartDashboard.putNumber("Arm Pivot Raw DC", rawDC);
        SmartDashboard.putNumber("Arm Pivot Angle deg", angle);

        return angle;
    }

    public boolean atBottom() {
        return getAngleDeg() <= REAL_MIN_ANGLE;
    }

    public boolean atTop() {
        return getAngleDeg() >= REAL_MAX_ANGLE;
    }



}
