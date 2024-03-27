package org.team100.frc2024.motion.amp;

import org.team100.frc2024.motion.GravityServo;
import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.DutyCycleEncoder100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A 1-dof arm driven by two separate motors with opposite phases.
 */
public class AmpSubsystem extends SubsystemBase implements Glassy {
    // ALERT! notice this very high current limit!!  ALERT! 
    private static final int kCurrentLimit = 80;

    private final String m_name;
    private final SysParam m_params;
    private final GravityServo ampAngleServo;
    private final CANSparkMax ampDrive;
    private final DutyCycleEncoder100 m_encoder;


 
    CANSparkMax m_motor;


    
    public AmpSubsystem(int pivotID) {
        m_encoder = new DutyCycleEncoder100("ANALOG ENCODER PIVOT", 3, 0.645439, true);
        m_name = Names.name(this);
        m_params = SysParam.neoPositionServoSystem(
                60,
                180,
                50)
                ;

        switch (Identity.instance) {
            case COMP_BOT:

                m_motor = new CANSparkMax(pivotID, MotorType.kBrushless);
                //TODO tune kV
                ampAngleServo = new GravityServo(
                    m_motor, 
                    30,
                    m_name, 
                    m_params, 
                    new PIDController(0.7, 0, 0), 
                    new TrapezoidProfile100(m_params.maxVelM_S(), m_params.maxAccelM_S2(), 0.05),
                    pivotID, 
                    0.02, 
                    -0.06, 
                    m_encoder,
                    new double[]{0, 0}
                );

                ampDrive = new CANSparkMax(33, MotorType.kBrushless);
                break;
            case BLANK:
            default:
                m_motor = new CANSparkMax(pivotID, MotorType.kBrushless);

                // ampAngleServo = new GravityServo(
                //     m_motor,
                //     5,
                //     m_name, 
                //     m_params, 
                //     new PIDController(3, 0, 0), 
                //     new TrapezoidProfile100(m_params.maxVelM_S(), m_params.maxAccelM_S2(), 0.05),
                //     pivotID, 
                //     0.02, 
                //     -0.06,
                //     new DutyCycleEncoder100("ANALOG ENCODER PIVOT", 2, 0.51, false),
                //     new double[]{0, 0}
                // );

                ampAngleServo = new GravityServo(
                    m_motor, 
                    30,
                    m_name, 
                    m_params, 
                    new PIDController(0.6, 0, 0), 
                    new TrapezoidProfile100(m_params.maxVelM_S(), m_params.maxAccelM_S2(), 0.05),
                    pivotID, 
                    0.02, 
                    -0.06, 
                    m_encoder,
                    new double[]{0, 0}
                );
                ampDrive = new CANSparkMax(33, MotorType.kBrushless);

                
        }
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
        ampDrive.set(value);
        // System.out.println("I AM BEING DRIVEN RIGHT NOW : " + value);
    }



    public void stop() {
        ampAngleServo.stop();
    }

    public Double getPositionRad() {
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

        // System.out.println("GET" + m_encoder.m_encoder.get());

        // System.out.println("Absolute" + m_encoder.m_encoder.getAbsolutePosition());
        // System.out.println("POSITION OFFSET" + m_encoder.m_encoder.getPositionOffset());
        // System.out.println("DISTANCE PER" + m_encoder.m_encoder.getDistancePerRotation());

        // System.out.println(m_encoder.m_encoder.get());

        

    }

    @Override
    public String getGlassName() {
        return "AMMMPPPPPPPPPP";
    }


    
}
