package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class Arm {

    public enum ArmPos{
        //TODO program the positions for the arm here
        HOME(-60, 0.0),
        PICKUP(-50, 0.0),
        PLACE_LOW(0, 1.0),
        PLACE_MID(10, 1.5),
        PLACE_HIGH(30, 2.0);

        public final double angle;
        public final double extension;
        ArmPos(double angle, double extension){
            this.angle = angle;
            this.extension = extension;
        }
    }

    //TODO figure this out by asking Tim or Jack or whoever
    // NOTE this may also not be exact since the spool diameter isn't consistent
    final double INCHES_PER_EXTENSION_MOTOR_ROT = 0.025;

    SparkMaxWrapper pivotMotor;
    ArmPositionSensor pivotSensor;

    SparkMaxWrapper extensionMotor;
    Solenoid pincherSolenoid;

    ArmPos curCmd = ArmPos.HOME;
    ArmPos prevCmd = ArmPos.HOME;

    boolean pinchCmd;

    public void setPinchCmd(boolean isPinched){
        pinchCmd = isPinched;
    }

    public void setPositionCmd(ArmPos newCmd){
        curCmd = newCmd;
    }

    public void update(){

        prevCmd = curCmd;
    }
}
