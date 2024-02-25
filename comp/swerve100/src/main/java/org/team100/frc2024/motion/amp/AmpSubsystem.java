package org.team100.frc2024.motion.amp;

import org.team100.frc2024.motion.GravityServo;
import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.encoder.AnalogEncoder100;
import org.team100.lib.encoder.SparkMaxEncoder;
import org.team100.lib.motion.components.PositionServoInterface;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.AngularVisualization;
import org.team100.lib.motion.simple.Positioning;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A 1-dof arm driven by two separate motors with opposite phases.
 */
public class AmpSubsystem extends SubsystemBase implements Positioning {
    // ALERT! notice this very high current limit!!  ALERT! 
    private static final int kCurrentLimit = 80;

    private final String m_name;
    private final SysParam m_params;
    private final GravityServo ampAngleServo;
    private final PWM ampDrive;

    private final AngularVisualization m_viz;
 
    CANSparkMax m_motor;


    
    public AmpSubsystem(int pivotID) {
        m_name = Names.name(this);
        m_params = SysParam.neoPositionServoSystem(
                45,
                100,
                100)
                ;

        switch (Identity.instance) {
            case COMP_BOT:

                m_motor = new CANSparkMax(pivotID, MotorType.kBrushless);
                //TODO tune kV
                ampAngleServo = new GravityServo(
                    m_motor, 
                    5,
                    "AMMMPPPPPPPPPP", 
                    m_params, 
                    new PIDController(10, 0, 0), 
                    new TrapezoidProfile100(m_params.maxVelM_S(), m_params.maxAccelM_S2(), 0.05),
                    pivotID, 
                    0.02, 
                    -0.06, 
                    new AnalogEncoder100("ANALOG ENCODER PIVOT", 2, 0.51)
                );

                ampDrive = new PWM(2);
                break;
            case BLANK:
            default:
                m_motor = new CANSparkMax(pivotID, MotorType.kBrushless);

                ampAngleServo = new GravityServo(
                    m_motor,
                    5,
                    m_name, 
                    m_params, 
                    new PIDController(1, 0, 0), 
                    new TrapezoidProfile100(m_params.maxVelM_S(), m_params.maxAccelM_S2(), 0.05),
                    pivotID, 
                    0.02, 
                    -0.06,
                    null
                );
                ampDrive = new PWM(2);

                
        }
        m_viz = new AngularVisualization(m_name, this);
    }

    /**
     * Set angle relative to the zero.
     * 
     * TODO: calibrate to the horizontal, reset the actual angle at the stop,
     * and/or use an absolute encoder.
     * 
     * @param value
     */
    public void setAmpPosition(double value) {
        ampAngleServo.setPositionWithSteadyState(value);
    }

    public void setDutyCycle(double value) {
        ampAngleServo.set(value);
    }

    public void reset() {
        ampAngleServo.reset();
    }

    public void driveFeeder(double value){
        ampDrive.setSpeed(-value);
    }



    public void stop() {
        ampAngleServo.stop();
    }

    @Override
    public double getPositionRad() {
        // return (ampAngleServoRight.getPosition() + ampAngleServoLeft.getPosition()) / 2;
        return ampAngleServo.getPosition();
    }

    public boolean inPosition() {
        // TODO get real values here
        return getPositionRad() < 0.75 * Math.PI && getPositionRad() > .5 * Math.PI;
    }

    @Override
    public void periodic() {
        ampAngleServo.periodic();
        m_viz.periodic();
        // ampDrive.setSpeed(-1);


        

    }
}
