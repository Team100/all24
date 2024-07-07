package org.team100.frc2024.motion.amp;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.frc2024.motion.GravityServo;
import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.encoder.turning.EncoderDrive;
import org.team100.lib.motor.SimulatedAngularMotor;
import org.team100.lib.motor.duty_cycle.NeoProxy;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Logger;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The pivot is independent from the feeder, so it's a separate subsystem.
 */
public class AmpPivot extends SubsystemBase implements Glassy {
    private final Logger m_logger;
    private final GravityServo ampAngleServo;

    public AmpPivot(Logger parent) {
        m_logger = parent.child(this);
        SysParam m_params = SysParam.neoPositionServoSystem(
                55,
                60,
                60);

        TrapezoidProfile100 profile = new TrapezoidProfile100(m_params.maxVelM_S(), m_params.maxAccelM_S2(), 0.05);
        double period = 0.02;

        switch (Identity.instance) {
            case COMP_BOT:
                ampAngleServo = new GravityServo(
                        new NeoProxy(m_logger, 2, IdleMode.kCoast, 30),
                        m_logger,
                        m_params,
                        new PIDController(0.8, 0, 0),
                        profile,
                        period,
                        new AS5048RotaryPositionSensor(m_logger, 3, 0.645439, EncoderDrive.INVERSE),
                        new double[] { 0, 0 });
                break;
            default:
                // For testing and simulation
                // motor speed is rad/s
                SimulatedAngularMotor simMotor = new SimulatedAngularMotor(
                        m_logger, 600);
                SimulatedRotaryPositionSensor simEncoder = new SimulatedRotaryPositionSensor(
                        m_logger,
                        simMotor,
                        75); // guess the gear ratio?
                ampAngleServo = new GravityServo(
                        simMotor,
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
        return ampAngleServo.getPositionRad();
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
