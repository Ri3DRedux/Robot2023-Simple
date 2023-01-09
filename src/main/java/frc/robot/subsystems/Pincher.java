package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pincher extends SubsystemBase{

    DoubleSolenoid pincherSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1); //TODO WHERE IS THIS PLUGGED INTO IT MIGHT NOT BE ZERO

    public void pinch() {
        pincherSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void release() {
        pincherSolenoid.set(DoubleSolenoid.Value.kReverse);
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
