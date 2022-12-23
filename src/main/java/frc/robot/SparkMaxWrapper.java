package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.util.Units;

/**
 * Small wrapper to sequence common things we will need to do with a spark max
 */
public class SparkMaxWrapper {
    
    private CANSparkMax m_motor;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;


    public SparkMaxWrapper(int can_id){
        
        m_motor = new CANSparkMax(can_id, MotorType.kBrushless);

        boolean success = false;

        while(!success){    
            var err0 = m_motor.restoreFactoryDefaults();
            var err1 = m_motor.setIdleMode(IdleMode.kCoast);
            var err2 = m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 19);// Status 0 = Motor output and Faults
            var err3 = m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 57);// Status 1 = Motor velocity & electrical data
            var err4 = m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65500);// Status 2 = Motor Position
            var err5 = m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65500);// Status 3 = Analog Sensor Input
            var err6 = m_motor.setSmartCurrentLimit(40); // 40 A current limit
            success = (err0 == REVLibError.kOk &&
                       err1 == REVLibError.kOk &&
                       err2 == REVLibError.kOk &&
                       err3 == REVLibError.kOk &&
                       err4 == REVLibError.kOk &&
                       err5 == REVLibError.kOk &&
                       err6 == REVLibError.kOk );
        
            if(!success){
                System.out.println("Configuration Failed, retrying....");
            }
        }
        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        
    }

    public void setInverted(boolean invert) {
        m_motor.setInverted(invert);
    }

    public void setClosedLoopGains(double p, double i, double d) {

        // I don't know why we need to do this, but it makes sim line up with real life.
        p /= 1000;

        //Convert to Rev units of RPM
        p = Units.radiansPerSecondToRotationsPerMinute(p);
        i = Units.radiansPerSecondToRotationsPerMinute(i);
        d = Units.radiansPerSecondToRotationsPerMinute(d);

        m_pidController.setP(p);
        m_pidController.setI(i);
        m_pidController.setD(d);
        m_pidController.setOutputRange(-1.0, 1.0);
    }

    public void setClosedLoopCmd(double velocityCmd_radpersec, double arbFF_V) {

        m_pidController.setReference(Units.radiansPerSecondToRotationsPerMinute(velocityCmd_radpersec), 
                                     CANSparkMax.ControlType.kVelocity,
                                     0,
                                     arbFF_V,
                                     SparkMaxPIDController.ArbFFUnits.kVoltage);
    }

    public void setVoltageCmd(double cmd_v) {
        m_motor.setVoltage(cmd_v);
    }

    public double getCurrent_A() {
        return m_motor.getOutputCurrent();
    }


    private double RPMtoDegPerSec(double rpmIn){ return rpmIn * 360 / 60.0; }

    public double getVelocity_radpersec() {
        return  Units.degreesToRadians(RPMtoDegPerSec(m_encoder.getVelocity()));
    }

    public void follow(SparkMaxWrapper leader) {
        this.m_motor.follow(leader.m_motor);
    }

    public double getPosition_rad() {
        return Units.rotationsToRadians(m_encoder.getPosition());
    }

    public double getAppliedVoltage_V() {
        return m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    }

    public void resetDistance() {
        m_encoder.setPosition(0.0);
    }
}
