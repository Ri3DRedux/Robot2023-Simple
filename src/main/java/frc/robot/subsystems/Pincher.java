package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Pincher extends SubsystemBase{

    DoubleSolenoid pincherSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1); //TODO WHERE IS THIS PLUGGED INTO IT MIGHT NOT BE ZERO
    public final TimeOfFlight m_tof = new TimeOfFlight(-1); // TODO get the CAN ID of the ToF

    private DoubleSolenoid.Value pinchPosition = DoubleSolenoid.Value.kForward;
    private DoubleSolenoid.Value releasePosition = DoubleSolenoid.Value.kReverse;

    Robot robot;

    public Pincher(Robot robot){
        this.robot = robot;

        m_tof.setRangingMode(RangingMode.Short, 25); // 40 times per second
    }

    public void pinch() {
        pincherSolenoid.set(pinchPosition);
    }

    public void release() {
        pincherSolenoid.set(releasePosition);
    }

    public boolean isPinched() {
        return pincherSolenoid.get() == pinchPosition;
    }

    public boolean isReleased() {
        return pincherSolenoid.get() == releasePosition;
    }

    public boolean havePossession() {
        return m_tof.getRange() < 40;
    }

    public void periodic() {
        SmartDashboard.putNumber("pincher/ToF", m_tof.getRange());
        SmartDashboard.putBoolean("pincher/havePossession", havePossession());
        SmartDashboard.putBoolean("pincher/isPinched", isPinched());
        SmartDashboard.putBoolean("pincher/isReleased", isReleased());
    }

    public CommandBase pinchCmd() {
        return new CommandBase() {
            public void initialize() {
                pinch();
            }

            public boolean isFinished() {
                return true;
            }
        }.andThen(Commands.waitSeconds(0));
    }

    
    public CommandBase releaseCmd() {
        return new CommandBase() {
            public void initialize() {
                release();
            }

            public boolean isFinished() {
                return true;
            }
        }.andThen(Commands.waitSeconds(0));
    }


}
