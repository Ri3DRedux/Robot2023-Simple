package frc.robot;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmPositionSensor {
    
    DutyCycleEncoder pivotEncoder;

    //TODO TWEAK THESE FOR WHAT THE REAL ROBOT IS
    public final double MIN_DUTY_CYCLE = 0.5;
    public final double MAX_DUTY_CYCLE = 1.0;
    public final double MIN_ANGLE = -20.0;
    public final double MAX_ANGLE = 160.0;

    public double getAngleDeg(){
        var rawDC = pivotEncoder.get();
        //Frac is 0 at lowest point, 1 at max extension
        var dcFrac = (rawDC - MIN_DUTY_CYCLE) / (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);
        var angle = MIN_ANGLE + dcFrac * (MAX_ANGLE - MIN_ANGLE);

        SmartDashboard.putNumber("Arm Pivot Raw DC", rawDC);
        SmartDashboard.putNumber("Arm Pivot Angle deg", angle);

        return angle;
    }

}
