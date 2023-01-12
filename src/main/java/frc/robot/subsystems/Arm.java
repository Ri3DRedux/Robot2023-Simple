package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Arm extends SubsystemBase {

    // REFERENCE FRAME NOTES
    //
    // Arm angle:
    // Zero is level with gravity ZOMG NOT ASCII, pointed toward the intake.
    // Negative is down toward the intake,
    // positive is up and over the top toward scoring positions
    //
    // Arm Extension:
    // 0 is fully retracted
    // positive is extended somewhat
    // negative is broken
    //
    // Pickup is on the intake side. Extension must be retracted first
    // otherwiswe we run into the intake.
    // THere will be some coordination needed with the intake as well, and to pinch
    // the pinchers

    public enum ArmPos {
        // TODO program the positions for the arm here
        HOME(-49, 0.0),
        PICKUP(-49, 0.0),
        PLACE_LOW(234, 5.0),
        PLACE_MID(189, 10.0),
        PLACE_HIGH(147, 12.0);

        public final double angle;
        public final double extension;

        ArmPos(double angle, double extension) {
            this.angle = angle;
            this.extension = extension;
        }
    }

    // TODO tuning strategy feedforward first
    // set all to zero - crank up kG till the arm feels weightless and doesn't kill
    // anyone
    // then tune kP and whatnot
    public final double pivot_kP = 0.0;
    public final double pivot_kI = 0.0;
    public final double pivot_kD = 0.0;
    public final double pivot_kG = 0.25; // Volts per cosine of angle or whatever that is in units does it look like I
                                         // know what I'm doing I've just got a BS in EE and you make me write code come
                                         // on man you get what you pay for and I ain't been paid squat except in coffee
                                         // and I really appreciate the coffee thanks #winning yay #bensbasementbunch

    private final double extension_kP = 0.0;
    private final double extension_kI = 0.0;
    private final double extension_kD = 0.0;

    // Extension Control. Simple bang-bang on NEO position converted to inches
    // Assumes we start with the mechanism fully retracted
    // Positive voltage should extend, negative voltage should retract
    // TODO figure this out by asking Tim or Jack or whoever
    // NOTE this may also not be exact since the spool diameter isn't consistent
    public CANSparkMax extensionMotor = new CANSparkMax(15, MotorType.kBrushless);
    public RelativeEncoder extensionEncoder;
    final double INCHES_PER_EXTENSION_MOTOR_ROT = 0.172;
    // final double INCHES_PER_EXTENSION_MOTOR_ROT = 1;
    public final double EXTENSION_ALLOWABLE_ERR_IN = 0.5;
    public final double EXTENSION_MAX_VOLTS = 4.0;
    public final double EXTENSION_MAX_INCHES = 12; // inches

    // final double EXTENSION_CONFLICT_MAX_ALLOWED = 0.5;
    // final double PIVOT_ANGLE_CONFLICT_THRESHOLD = 0.0; // total guess but this is
    // the angle or length where mechanical
    // conflict happpens

    public CANSparkMax pivotMotor = new CANSparkMax(10, MotorType.kBrushless);
    public PivotSensor pivotSensor = new PivotSensor(0);

    // ArmPos curCmd = ArmPos.HOME;
    // ArmPos prevCmd = ArmPos.HOME;
    // double extensionPosition_in;

    private Robot robot;

    public PIDController pivotPIDController = new PIDController(pivot_kP, pivot_kI, pivot_kD);
    public PIDController extensionPIDController = new PIDController(extension_kP, extension_kI, extension_kD);

    public Arm(Robot robot) {
        this.robot = robot;

        // extensionMotor.restoreFactoryDefaults();
        extensionMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.setSmartCurrentLimit(40);
        extensionMotor.setInverted(true);
        // extensionMotor.burnFlash();

        extensionEncoder = extensionMotor.getEncoder();
        // extensionEncoder.setPositionConversionFactor(INCHES_PER_EXTENSION_MOTOR_ROT);

        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setSmartCurrentLimit(40);
        pivotMotor.setInverted(false);
        pivotMotor.burnFlash();

        extensionPIDController.setTolerance(EXTENSION_ALLOWABLE_ERR_IN);
        pivotPIDController.setTolerance(EXTENSION_ALLOWABLE_ERR_IN);
    }

    public void stop() {
        pivotMotor.stopMotor();
        extensionMotor.stopMotor();
    }

    public boolean movePivot(double volts) {
        // positive volts up
        // return success (true if can move, false if move was blocked)
        if (volts > 0) {
            if (pivotSensor.atTop() || robot.intake.isUp()) {
                pivotMotor.stopMotor();
                return false;
            } else {
                pivotMotor.setVoltage(volts);
                return true;
            }
        } else {
            if (pivotSensor.atBottom()
                    || (getExtensionPosition() > 5 && ! isPivotOutOfTheWayOfTheIntake())
                    || (robot.intake.isUp() && ! isPivotOutOfTheWayOfTheIntake())) {
                pivotMotor.stopMotor();
                return false;
            } else {
                pivotMotor.setVoltage(volts);
                return true;
            }
        }
    }

    public void movePivotUnsafe(double volts) {
        pivotMotor.setVoltage(volts);
    }

    public boolean moveExtension(double volts) {
        // Positive voltage should extend, negative voltage should retract
        // return success (true if can move, false if move was blocked)
        if (volts > 0) {
            if (false && (extensionEncoder.getPosition() >= EXTENSION_MAX_INCHES || !isPivotOutOfTheWayOfTheIntake())) {
                extensionMotor.stopMotor();
                return false;
            } else {
                extensionMotor.setVoltage(volts);
                return true;
            }
        } else {
            if (getExtensionPosition() <= 0.5) {
                extensionMotor.stopMotor();
                return false;
            } else {
                extensionMotor.setVoltage(volts);
                return true;
            }
        }
    }

    public void moveExtensionUnsafe(double volts) {
        extensionMotor.setVoltage(volts);
    }

    public double getExtensionPosition() {
        return extensionEncoder.getPosition();
    }

    public double getPivotPosition() {
        return pivotSensor.getAngleDeg();
    }

    public boolean isPivotOutOfTheWayOfTheIntake() {
        return getPivotPosition() > -17;
    }

    public void periodic() {
        SmartDashboard.putNumber("arm/pivot angle", getPivotPosition());
        SmartDashboard.putNumber("arm/extension position", getExtensionPosition());
        SmartDashboard.putBoolean("arm/isPivotOutOfTheWayOfTheIntake", isPivotOutOfTheWayOfTheIntake());
        SmartDashboard.putBoolean("arm/pivotSensor/atTop", pivotSensor.atTop());
        SmartDashboard.putBoolean("arm/pivotSensor/atBottom", pivotSensor.atBottom());
        SmartDashboard.putNumber("arm/pivot voltage", 12 * pivotMotor.get());
        SmartDashboard.putNumber("arm/velocity", pivotMotor.getEncoder().getVelocity());
    
    }

    public CommandBase pivotArmToCmd(double degrees) {
        return new CommandBase() {
            public void initialize() {
                pivotPIDController.reset();
            }

            public void execute() {
                var pivotFeedbackV = pivotPIDController.calculate(getPivotPosition(), degrees);
                // Feedforward - delete gravity
                var pivotFeedforwardV = Math.cos(Units.degreesToRadians(getPivotPosition())) * pivot_kG;

                var couldWeMove = movePivot(pivotFeedforwardV + pivotFeedbackV);

                if (!couldWeMove) {
                    pivotPIDController.reset();
                }
            }

            public boolean isFinished() {
                return pivotPIDController.atSetpoint();
            }
        };
    }

    // A basic structure that should be straightforward and not completely insane to
    // understand for a command
    public CommandBase extendExtensionToCmd(double inches) {
        return new CommandBase() {
            public void initialize() {
                extensionPIDController.reset();
            }

            public void execute() {
                double volts = extensionPIDController.calculate(getExtensionPosition(), inches);
                volts = MathUtil.clamp(volts, -EXTENSION_MAX_VOLTS, EXTENSION_MAX_VOLTS);

                var couldWeMove = moveExtension(volts);

                if (!couldWeMove) {
                    extensionPIDController.reset();
                }
            }

            public boolean isFinished() {
                return extensionPIDController.atSetpoint();
            }
        };
    }

    public CommandBase moveArmToCmd(ArmPos position) {
        return new ParallelCommandGroup(
                extendExtensionToCmd(position.extension),
                pivotArmToCmd(position.angle));
    }

    // public void setPositionCmd(ArmPos newCmd) {
    // curCmd = newCmd;
    // }

    // public void update() {

    // // Read Sensors
    // extensionPosition_in = getExtensionPosition(); // already converted

    // double curPivotPos_deg = pivotSensor.getAngleDeg();

    // ///////////////////////////////////////////////////////
    // // Extension Control

    // double desExtensionPosition_in = extensionPosition_in; // default to not
    // change position

    // // conflict management - cap the maximum extension if we're below the angle
    // // where conflicts happen
    // if (curPivotPos_deg < PIVOT_ANGLE_CONFLICT_THRESHOLD) {
    // desExtensionPosition_in = Math.min(curCmd.extension,
    // EXTENSION_CONFLICT_MAX_ALLOWED);
    // } else {
    // desExtensionPosition_in = curCmd.extension;
    // }

    // // Saturated PID control
    // double motorCmdV = extensionPIDController.calculate(extensionPosition_in,
    // desExtensionPosition_in);
    // motorCmdV = Math.min(motorCmdV, EXTENSION_MAX_VOLTS);
    // motorCmdV = Math.max(motorCmdV, -1.0 * EXTENSION_MAX_VOLTS);
    // boolean atTarget = extensionPIDController.atSetpoint();

    // SmartDashboard.putNumber("Arm Extension Desired in", extensionPosition_in);
    // SmartDashboard.putNumber("Arm Extension Desired Extension",
    // desExtensionPosition_in);
    // SmartDashboard.putNumber("Arm Cmd Voltage", motorCmdV);

    // // moveEx

    // ///////////////////////////////////////////////////////
    // // Angle Control

    // double desPivotPos_deg = curPivotPos_deg; // default to not change position

    // // conflict management - cap the angle if we aren't retracted on the
    // extension
    // // enough
    // if (extensionPosition_in > EXTENSION_CONFLICT_MAX_ALLOWED) {
    // desPivotPos_deg = Math.max(curCmd.angle, PIVOT_ANGLE_CONFLICT_THRESHOLD);
    // } else {
    // desPivotPos_deg = curCmd.angle;
    // }

    // // PID Control
    // // Positive voltage should move the arm away from the intake and toward
    // scoring
    // // positions
    // double pivotFeedbackV = pivotPIDController.calculate(curPivotPos_deg,
    // desPivotPos_deg);

    // // Feedforward - delete gravity
    // double pivotFeedforwardV = Math.cos(Units.degreesToRadians(curPivotPos_deg))
    // * pivot_kG;

    // SmartDashboard.putNumber("Arm Pivot Angle Actual deg", curPivotPos_deg);
    // SmartDashboard.putNumber("Arm Pivot Angle Desired deg", desPivotPos_deg);
    // SmartDashboard.putNumber("Arm Pivot Angle FB Cmd Voltage", pivotFeedbackV);
    // SmartDashboard.putNumber("Arm Pivot Angle FF Cmd Voltage",
    // pivotFeedforwardV);

    // movePivot(pivotFeedforwardV + pivotFeedbackV);

    // prevCmd = curCmd;
    // }

}

// // A basic structure that should be straightforward and not completely insane
// to understand for a command
// public CommandBase moveToCmd(double degrees) {
// return new CommandBase() {
// public void initialize() {
// // runs once when the command starts
// }

// public void execute() {
// // loops until isFinished() == true
// }

// public boolean isFinished() {
// // end condition
// return true;
// }
// };
// }