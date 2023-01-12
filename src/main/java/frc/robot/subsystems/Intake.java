package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

    public WPI_TalonSRX rollerMotor = new WPI_TalonSRX(11);
    Solenoid extendSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1); // TODO this

    private boolean upPosition = true;
    private boolean downPosition = false;

    Robot robot;

    public Intake(Robot robot) {
        this.robot = robot;

        setDefaultCommand(this.run(() -> {
            stop();
        }));
    }

    public void intake() {
        intake(12);
    }

    public void intake(double volts) {
        rollerMotor.setVoltage(volts);
        
    }

    public void outtake() {
        outtake(12);
    }

    public void stop() {
        rollerMotor.stopMotor();
    }

    public void outtake(double volts) {
        rollerMotor.setVoltage(-volts);
    }

    public void down() {
        extendSolenoid.set(downPosition);
    }

    public void up() {
        extendSolenoid.set(upPosition);
    }

    public boolean isUp() {
        return extendSolenoid.get() == upPosition;
    }

    public boolean isDown() {
        return extendSolenoid.get() == downPosition;
    }

    public void periodic() {
        SmartDashboard.putBoolean("intake/isUp", isUp());
        SmartDashboard.putBoolean("intake/isDown", isDown());

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
