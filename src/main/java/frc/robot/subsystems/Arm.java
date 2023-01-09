package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SparkMaxWrapper;

public class Arm extends SubsystemBase{

    //REFERENCE FRAME NOTES
    //
    // Arm angle: 
    // Zero is level with gravity 👍 ZOMG NOT ASCII, pointed toward the intake. 
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
    // THere will be some coordination needed with the intake as well, and to pinch the pinchers

    public enum ArmPos{
        //TODO program the positions for the arm here
        HOME(-60, 0.0), 
        PICKUP(-50, 0.0),
        PLACE_LOW(215, 1.0),
        PLACE_MID(180, 1.5),
        PLACE_HIGH(150, 2.0);

        public final double angle;
        public final double extension;
        ArmPos(double angle, double extension){
            this.angle = angle;
            this.extension = extension;
        }
    }

//TODO tuning strategy feedforward first
// set all to zero - crank up kG till the arm feels weightless and doesn't kill anyone
// then tune kP and whatnot
    private final double pivot_kP = 1.0;
    private final double pivot_kI = 0.0;
    private final double pivot_kD = 0.0;
    private final double pivot_kG = 0.0; // Volts per cosine of angle or whatever that is in units does it look like I know what I'm doing I've just got a BS in EE and you make me write code come on man you get what you pay for and I ain't been paid squat except in coffee and I really appreciate the coffee thanks #winning yay #bensbasementbunch
    
    private final double extension_kP = 1.0;
    private final double extension_kI = 0.0;
    private final double extension_kD = 0.0;

    // Extension Control. Simple bang-bang on NEO position converted to inches
    // Assumes we start with the mechanism fully retracted
    // Positive voltage should extend, negative voltage should retract
    //TODO figure this out by asking Tim or Jack or whoever
    // NOTE this may also not be exact since the spool diameter isn't consistent
    CANSparkMax extensionMotor;
    RelativeEncoder extensionEncoder;
    final double INCHES_PER_EXTENSION_MOTOR_ROT = 0.025;
    final double EXTENSION_ALLOWABLE_ERR_IN = 0.5;
    final double EXTENSION_MAX_V = 4.0;

    final double EXTENSION_CONFLICT_MAX_ALLOWED = 0.5;
    final double PIVOT_ANGLE_CONFLICT_THRESHOLD = 0.0; //total guess but this is the angle or length where mechanical conflict happpens

    CANSparkMax pivotMotor;
    PivotSensor pivotSensor;

    ArmPos curCmd = ArmPos.HOME;
    ArmPos prevCmd = ArmPos.HOME;
    double extensionPosition_in;

    Robot robot;

    PIDController pivotPIDController = new PIDController(pivot_kP, pivot_kI, pivot_kD);
    PIDController extensionPIDController = new PIDController(extension_kP, extension_kI, extension_kD);


    public Arm(Robot robot) {
        this.robot = robot;
        
        extensionMotor.restoreFactoryDefaults();
        extensionMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.setSmartCurrentLimit(40);
        extensionMotor.setInverted(false);
        extensionMotor.burnFlash();

        extensionEncoder = extensionMotor.getEncoder();
        extensionEncoder.setPositionConversionFactor(INCHES_PER_EXTENSION_MOTOR_ROT)

        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setSmartCurrentLimit(40);
        pivotMotor.setInverted(false);
        pivotMotor.burnFlash();

        extensionPIDController.setTolerance(EXTENSION_ALLOWABLE_ERR_IN);
        pivotPIDController.setTolerance(EXTENSION_ALLOWABLE_ERR_IN);
    }

    public void setPositionCmd(ArmPos newCmd){
        curCmd = newCmd;
    }

    public void stop(){
        pivotMotor.stopMotor();
        extensionMotor.stopMotor();
    }
    
    public void movePivot(double volts){
        // positive volts up

        if (volts > 0) {
            if (pivotSensor.atTop() || ) {
                pivotMotor.stopMotor();
            } else {
                pivotMotor.setVoltage(volts);
            }
        } else {
            if (pivotSensor.atBottom()) {
                pivotMotor.stopMotor();
            } else {
                pivotMotor.setVoltage(volts);
            }
        }
    }

    public void moveExtension(double volts){
        // Positive voltage should extend, negative voltage should retract

    }

    public double getElevatorPosition() {
        return extensionEncoder.getPosition();
    }

    public double getPivotPosition() {
        return pivotSensor.getAngleDeg();
    }

    public void update(){

        //Read Sensors
        double curExtensionPos_rotations = getElevatorPosition();
        extensionPosition_in = curExtensionPos_rotations * INCHES_PER_EXTENSION_MOTOR_ROT;
        double curPivotPos_deg = pivotSensor.getAngleDeg();

        ///////////////////////////////////////////////////////
        // Extension Control
        
        double desExtensionPosition_in = extensionPosition_in; // default to not change position

        //conflict management - cap the maximum extension if we're below the angle where conflicts happen
        if(curPivotPos_deg < PIVOT_ANGLE_CONFLICT_THRESHOLD){
            desExtensionPosition_in = Math.min(curCmd.extension, EXTENSION_CONFLICT_MAX_ALLOWED);
        } else {
            desExtensionPosition_in = curCmd.extension;
        }

        //Saturated PID control
        double motorCmdV = extensionPIDController.calculate(extensionPosition_in, desExtensionPosition_in);
        motorCmdV = Math.min(motorCmdV, EXTENSION_MAX_V);
        motorCmdV = Math.max(motorCmdV, -1.0 * EXTENSION_MAX_V);
        boolean atTarget = extensionPIDController.atSetpoint();

        SmartDashboard.putNumber("Arm Extension Desired in", extensionPosition_in);       
        SmartDashboard.putNumber("Arm Extension Desired Extension", desExtensionPosition_in);
        SmartDashboard.putNumber("Arm Cmd Voltage", motorCmdV);

        extensionMotor.setVoltage(motorCmdV);

        ///////////////////////////////////////////////////////
        // Angle Control
        
        double desPivotPos_deg = curPivotPos_deg; // default to not change position

        //conflict management - cap the angle if we aren't retracted on the extension enough
        if(extensionPosition_in > EXTENSION_CONFLICT_MAX_ALLOWED){
            desPivotPos_deg = Math.max(curCmd.angle, PIVOT_ANGLE_CONFLICT_THRESHOLD);
        } else {
            desPivotPos_deg = curCmd.angle;
        }

        //PID Control
        // Positive voltage should move the arm away from the intake and towared scoring positions
        double pivotFeedbackV = extensionPIDController.calculate(curPivotPos_deg, desPivotPos_deg);
        
        //Feedforward - delete gravity
        double pivotFeedforwardV = Math.cos(Units.degreesToRadians(curPivotPos_deg)) * pivot_kG;

        SmartDashboard.putNumber("Arm Pivot Angle Actual deg", extensionPosition_in);       
        SmartDashboard.putNumber("Arm Pivot Angle Desired deg", desPivotPos_deg);
        SmartDashboard.putNumber("Arm Pivot Angle FB Cmd Voltage", pivotFeedbackV);
        SmartDashboard.putNumber("Arm Pivot Angle FF Cmd Voltage", pivotFeedforwardV);

        if(!atTarget)
        {
            movePivot(pivotFeedforwardV + pivotFeedbackV);
        }

        prevCmd = curCmd;
    }
    
    // A basic structure that should be straightforward and not completely insane to understand for a command
    public CommandBase moveToCmd(double degrees) {
        return new CommandBase() {
            public void initialize() {
                // runs once when the command starts
            }

            public void execute() {
                // loops until isFinished() == true
            }

            public boolean isFinished() {
                // end condition
                return true;
            }
        };
    }

}
