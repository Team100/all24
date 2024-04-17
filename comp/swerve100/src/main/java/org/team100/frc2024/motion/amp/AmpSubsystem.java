package org.team100.frc2024.motion.amp;

import org.team100.frc2024.motion.GravityServo;
import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.DutyCycleEncoder100;
import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.duty_cycle.NeoProxy;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpSubsystem extends SubsystemBase implements Glassy {
    private final String m_name;
    private final SysParam m_params;
    private final GravityServo ampAngleServo;
    private final Motor100<Distance100> ampDrive;

    public AmpSubsystem() {
        m_name = Names.name(this);
        m_params = SysParam.neoPositionServoSystem(
                55,
                60,
                60);

        TrapezoidProfile100 profile = new TrapezoidProfile100(m_params.maxVelM_S(), m_params.maxAccelM_S2(), 0.05);
        double period = 0.02;

        switch (Identity.instance) {
            case COMP_BOT:
                ampAngleServo = new GravityServo(
                        new NeoProxy(m_name, 2, false, 30),
                        m_name,
                        m_params,
                        new PIDController(0.8, 0, 0),
                        profile,
                        period,
                        new DutyCycleEncoder100("ANALOG ENCODER PIVOT", 3, 0.645439, true),
                        new double[] { 0, 0 });
                ampDrive = new NeoProxy(m_name, 33, true, 40);
                break;
            default:
                // For testing and simulation
                // motor speed is rad/s
                SimulatedMotor<Distance100> simMotor = new SimulatedMotor<>(m_name, 600);
                SimulatedEncoder<Distance100> simEncoder = new SimulatedEncoder<>(
                        m_name,
                        simMotor,
                        75, // guess the gear ratio?
                        -Double.MAX_VALUE,
                        Double.MAX_VALUE);
                ampAngleServo = new GravityServo(
                        simMotor,
                        m_name,
                        m_params,
                        new PIDController(0.7, 0, 0),
                        profile,
                        period,
                        simEncoder,
                        new double[] { 0, 0 });
                        // motor speed is rad/s
                ampDrive = new SimulatedMotor<>(m_name, 600);
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

    /** 1 = outtake, -1 = intake */
    public void driveFeeder(double value) {
        ampDrive.setDutyCycle(value);
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
