package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Pincher extends SubsystemBase{

    DoubleSolenoid pincherSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1); //TODO WHERE IS THIS PLUGGED INTO IT MIGHT NOT BE ZERO

    private DoubleSolenoid.Value pinchPosition = DoubleSolenoid.Value.kForward;
    private DoubleSolenoid.Value releasePosition = DoubleSolenoid.Value.kReverse;

    Robot robot;

    public Pincher(Robot robot){
        this.robot = robot;
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
