package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    public CANSparkMax m_motor = new CANSparkMax(-1, MotorType.kBrushed);
    public RelativeEncoder m_encoder;
    private static final double SCALE_FACTOR = 1;

    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public PIDController pidController = new PIDController(kP, kI, kD);

    public Arm() {

        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setSmartCurrentLimit(40);
        m_motor.setInverted(true);
        m_motor.burnFlash();

        m_encoder = m_motor.getEncoder();
        m_encoder.setPositionConversionFactor(SCALE_FACTOR);

        setDefaultCommand(Commands.runOnce(() -> {
            stop();
        }, this));

    }

    public void periodic() {
        SmartDashboard.putNumber("Arm Position", m_encoder.getPosition());
    }

    public void move(double volts) {
        m_motor.setVoltage(volts);
    }

    public void stop() {
        move(0);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public CommandBase moveToCmd(double degrees) {
        class MoveToCommand extends CommandBase {
            public void initialize() {
                pidController.reset();
            }

            public void execute() {
                move(pidController.calculate(getPosition(), degrees));
            }

            public boolean isFinished() {
                return pidController.atSetpoint();
            }
        }
        return new MoveToCommand();
    }
}
