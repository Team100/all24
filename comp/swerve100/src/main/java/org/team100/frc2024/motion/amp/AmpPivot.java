package org.team100.frc2024.motion.amp;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.frc2024.motion.GravityServo;
import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.DutyCycleEncoder100;
import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.duty_cycle.NeoProxy;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The pivot is independent from the feeder, so it's a separate subsystem.
 */
public class AmpPivot extends SubsystemBase implements Glassy {
    private final String m_name;
    private final Logger m_logger;
    private final GravityServo ampAngleServo;

    public AmpPivot() {
        m_name = Names.name(this);
        m_logger = Telemetry.get().rootLogger(m_name);
        SysParam m_params = SysParam.neoPositionServoSystem(
                55,
                60,
                60);

        TrapezoidProfile100 profile = new TrapezoidProfile100(m_params.maxVelM_S(), m_params.maxAccelM_S2(), 0.05);
        double period = 0.02;

        switch (Identity.instance) {
            case COMP_BOT:
                ampAngleServo = new GravityServo(
                        new NeoProxy(m_name, m_logger, 2, IdleMode.kCoast, 30),
                        m_name,
                        m_logger,
                        m_params,
                        new PIDController(0.8, 0, 0),
                        profile,
                        period,
                        new DutyCycleEncoder100("ANALOG ENCODER PIVOT", m_logger, 3, 0.645439, true),
                        new double[] { 0, 0 });
                break;
            default:
                // For testing and simulation
                // motor speed is rad/s
                SimulatedMotor<Distance100> simMotor = new SimulatedMotor<>(
                        m_name, m_logger, 600);
                SimulatedEncoder<Distance100> simEncoder = new SimulatedEncoder<>(
                        m_name,
                        m_logger,
                        simMotor,
                        75, // guess the gear ratio?
                        -Double.MAX_VALUE,
                        Double.MAX_VALUE);
                ampAngleServo = new GravityServo(
                        simMotor,
                        m_name,
                        m_logger,
                        m_params,
                        new PIDController(0.7, 0, 0),
                        profile,
                        period,
                        simEncoder,
                        new double[] { 0, 0 });
        }
    }

    public void setAmpPosition(double value) {
        ampAngleServo.setPositionWithSteadyState(value);
    }

    public void setDutyCycle(double value) {
        ampAngleServo.set(value);
    }

    /** Zeros controller errors, sets setpoint to current position. */
    public void reset() {
        ampAngleServo.reset();
    }

    public void stop() {
        ampAngleServo.stop();
    }

    public OptionalDouble getPositionRad() {
        return ampAngleServo.getPosition();
    }

    public Optional<Boolean> inPosition() {
        OptionalDouble position = getPositionRad();
        if (position.isEmpty())
            return Optional.empty();
        return Optional.of(
                position.getAsDouble() < 0.75 * Math.PI
                        && position.getAsDouble() > .5 * Math.PI);
    }

    @Override
    public String getGlassName() {
        return "Amp Pivot";
    }
}
