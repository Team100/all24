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

public class AmpSubsystem extends SubsystemBase implements Glassy {
    private final String m_name;
    private final SysParam m_params;
    private final GravityServo ampAngleServo;
    private final CANSparkMax ampDrive;
    private final DutyCycleEncoder100 m_encoder;
    private final CANSparkMax m_motor;

    public AmpSubsystem() {
        m_encoder = new DutyCycleEncoder100("ANALOG ENCODER PIVOT", 3, 0.645439, true);
        m_name = Names.name(this);
        m_params = SysParam.neoPositionServoSystem(
                55,
                60,
                60);

        switch (Identity.instance) {
            case COMP_BOT:
                m_motor = new CANSparkMax(2, MotorType.kBrushless);
                ampAngleServo = new GravityServo(
                        m_motor,
                        30,
                        m_name,
                        m_params,
                        new PIDController(0.8, 0, 0),
                        new TrapezoidProfile100(m_params.maxVelM_S(), m_params.maxAccelM_S2(), 0.05),
                        0.02,
                        m_encoder,
                        new double[] { 0, 0 });
                ampDrive = new CANSparkMax(33, MotorType.kBrushless);
                break;
            case BLANK:
            default:
                m_motor = new CANSparkMax(2, MotorType.kBrushless);
                ampAngleServo = new GravityServo(
                        m_motor,
                        30,
                        m_name,
                        m_params,
                        new PIDController(0.7, 0, 0),
                        new TrapezoidProfile100(m_params.maxVelM_S(), m_params.maxAccelM_S2(), 0.05),
                        0.02,
                        m_encoder,
                        new double[] { 0, 0 });
                ampDrive = new CANSparkMax(33, MotorType.kBrushless);
        }
    }

    public void setAmpPosition(double value) {
        ampAngleServo.setPositionWithSteadyState(value);
    }

    public void setDutyCycle(double value) {
        ampAngleServo.set(value);
    }

    public void reset() {
        ampAngleServo.reset();
    }

    public void driveFeeder(double value) {
        ampDrive.set(value);
    }

    public void stop() {
        ampAngleServo.stop();
    }

    public Double getPositionRad() {
        return ampAngleServo.getPosition();
    }

    public boolean inPosition() {
        return getPositionRad() < 0.75 * Math.PI && getPositionRad() > .5 * Math.PI;
    }

    @Override
    public String getGlassName() {
        return "AMMMPPPPPPPPPP";
    }

}
