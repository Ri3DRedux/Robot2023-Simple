package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

    public PWMTalonSRX rollerMotor;
    Solenoid extendSolenoid;

    Robot robot;

    public Intake(Robot robot) {
        this.robot = robot;
    }
    
}
