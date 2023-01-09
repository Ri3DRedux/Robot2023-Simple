package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

    public PWMTalonSRX rollerMotor = new PWMTalonSRX(-1);
    DoubleSolenoid extendSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0); // TODO this

    private DoubleSolenoid.Value upPosition = DoubleSolenoid.Value.kForward;
    private DoubleSolenoid.Value downPosition = DoubleSolenoid.Value.kReverse;

    Robot robot;

    public Intake(Robot robot) {
        this.robot = robot;
    }

    public void intake() {
        intake(0.5);
    }

    public void intake(double volts) {
        rollerMotor.setVoltage(volts);
    }

    public void outtake() {
        outtake(0.5);
    }

    public void outtake(double volts) {
        rollerMotor.setVoltage(-volts);
    }

    public void down() {
        extendSolenoid.set(upPosition);
    }

    public void up() {
        extendSolenoid.set(downPosition);
    }

    public boolean isUp() {
        return extendSolenoid.get() == upPosition;
    }

    public boolean isDown() {
        return extendSolenoid.get() == downPosition;
    }

    public CommandBase upCmd() {
        return new CommandBase() {
            public void initialize() {
                up();
            }

            public boolean isFinished() {
                return true;
            }
        }.andThen(Commands.waitSeconds(0));
    }

    public CommandBase downCmd() {
        return new CommandBase() {
            public void initialize() {
                down();
            }

            public boolean isFinished() {
                return true;
            }
        }.andThen(Commands.waitSeconds(0));
    }

    public CommandBase upAfterArmLeavesCmd() {
        return new WaitUntilCommand(() -> {
            return robot.arm.isPivotOutOfTheWayOfTheIntake();
        }).andThen(upCmd());
    }

}
