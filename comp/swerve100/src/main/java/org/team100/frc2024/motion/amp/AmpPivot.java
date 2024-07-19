package org.team100.frc2024.motion.amp;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.frc2024.motion.GravityServo;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motor.CANSparkMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.Dashpot;
import org.team100.lib.profile.JerkLimiter;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.SupplierLogger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The pivot is independent from the feeder, so it's a separate subsystem.
 */
public class AmpPivot extends SubsystemBase implements Glassy {
    private static final int kCurrentLimit = 30;
    private static final int kCanId = 2;
    private static final double kMaxJerk = 5;
    private static final double kMaxAccel = 5;
    private static final double kMaxVelocity = 5;
    private static final double kGearRatio = 55;
    private final SupplierLogger m_logger;
    private final GravityServo ampAngleServo;

    public AmpPivot(SupplierLogger parent) {
        m_logger = parent.child(this);

        Profile100 profile = getProfile();
        double period = 0.02;
        PIDController controller = new PIDController(0.8, 0, 0);

        switch (Identity.instance) {
            case COMP_BOT:
                CANSparkMotor motor = new NeoCANSparkMotor(
                        m_logger,
                        kCanId,
                        MotorPhase.FORWARD,
                        kCurrentLimit,
                        Feedforward100.makeNeo(),
                        new PIDConstants(0, 0, 0));
                RotaryMechanism mech = new RotaryMechanism(
                        motor,
                        new CANSparkEncoder(m_logger, motor),
                        kGearRatio);
                RotaryPositionSensor encoder = new AS5048RotaryPositionSensor(
                        m_logger, 3, 0.645439, EncoderDrive.INVERSE);
                ampAngleServo = new GravityServo(
                        mech,
                        m_logger,
                        controller,
                        profile,
                        period,
                        encoder);
                break;
            default:
                // For testing and simulation
                double freeSpeedRad_S = 600;
                SimulatedBareMotor simMotor = new SimulatedBareMotor(
                        m_logger, freeSpeedRad_S);
                RotaryMechanism simMech = new RotaryMechanism(
                        simMotor,
                        new SimulatedBareEncoder(m_logger, simMotor),
                        kGearRatio);
                RotaryPositionSensor simEncoder = new SimulatedRotaryPositionSensor(
                        m_logger, simMech);
                ampAngleServo = new GravityServo(
                        simMech,
                        m_logger,
                        controller,
                        profile,
                        period,
                        simEncoder);
        }
    }

    private Profile100 getProfile() {
        Profile100 fast = new TrapezoidProfile100(kMaxVelocity, kMaxAccel, 0.05);
        if (Experiments.instance.enabled(Experiment.FancyProfile)) {
            Profile100 slow = new TrapezoidProfile100(0.1 * kMaxVelocity, 0.1 * kMaxAccel, 0.05);
            // TODO: make sure this distance is enough
            final double distance = 0.2;
            Dashpot d = new Dashpot(fast, slow, distance, 0.1 * kMaxVelocity);
            return new JerkLimiter(d, kMaxJerk);
        }
        return fast;
    }

    public void setAmpPosition(double value) {
        ampAngleServo.setPosition(value);
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
