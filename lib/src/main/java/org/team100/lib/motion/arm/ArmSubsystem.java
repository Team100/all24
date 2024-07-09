package org.team100.lib.motion.arm;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.lib.commands.Subsystem100;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.visualization.ArmVisualization;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;

/**
 * Arm mechanism with two joints.
 */
public class ArmSubsystem extends Subsystem100 {
    private static final double kFilterTimeConstantS = 0.06;
    private static final double kFilterPeriodS = 0.02;

    private final Logger m_logger;
    private final LinearFilter m_lowerMeasurementFilter;
    private final LinearFilter m_upperMeasurementFilter;
    private final RotaryMechanism m_lowerArmMotor;
    private final RotaryMechanism m_upperArmMotor;
    private final RotaryPositionSensor m_lowerArmEncoder;
    private final RotaryPositionSensor m_upperArmEncoder;
    private final ArmVisualization m_viz;

    private ArmAngles m_previousPosition;

    // use the factory to instantiate
    /**
     * @param lowerMotor
     * @param lowerEncoder Lower arm angle (radians), 0 up, positive forward.
     * @param upperMotor
     * @param upperEncoder Upper arm angle (radians), 0 up, positive forward.
     */
    ArmSubsystem(
            Logger parent,
            RotaryMechanism lowerMotor,
            RotaryPositionSensor lowerEncoder,
            RotaryMechanism upperMotor,
            RotaryPositionSensor upperEncoder) {
        m_logger = parent.child(this);

        m_lowerMeasurementFilter = LinearFilter.singlePoleIIR(kFilterTimeConstantS, kFilterPeriodS);
        m_upperMeasurementFilter = LinearFilter.singlePoleIIR(kFilterTimeConstantS, kFilterPeriodS);

        m_lowerArmMotor = lowerMotor;
        m_lowerArmEncoder = lowerEncoder;
        m_upperArmMotor = upperMotor;
        m_upperArmEncoder = upperEncoder;

        Optional<ArmAngles> position = getPosition();
        if (position.isPresent())
            m_previousPosition = position.get();
        m_viz = new ArmVisualization(this);
    }

    /** Arm angles (radians), 0 up, positive forward. */
    public Optional<ArmAngles> getPosition() {
        OptionalDouble lowerPosition = m_lowerArmEncoder.getPositionRad();
        OptionalDouble upperPosition = m_upperArmEncoder.getPositionRad();
        if (lowerPosition.isEmpty() || upperPosition.isEmpty())
            return Optional.empty();
        ArmAngles position = new ArmAngles(
                MathUtil.angleModulus(m_lowerMeasurementFilter.calculate(lowerPosition.getAsDouble())),
                MathUtil.angleModulus(m_upperMeasurementFilter.calculate(upperPosition.getAsDouble())));
        m_logger.logArmAngles(Level.TRACE, "position", () -> position);
        return Optional.of(position);
    }

    /** Joint velocities in radians per second. */
    public Optional<ArmAngles> getVelocity() {
        Optional<ArmAngles> position = getPosition();
        if (position.isEmpty())
            return Optional.empty();
        double th1 = position.get().th1 - m_previousPosition.th1;
        double th2 = position.get().th2 - m_previousPosition.th2;
        m_previousPosition = position.get();
        ArmAngles velocity = new ArmAngles(th1 * 50, th2 * 50);
        m_logger.logArmAngles(Level.TRACE, "velocity", () -> velocity);
        return Optional.of(velocity);
    }

    /**
     * Set motor controller duty cycle in range [-1, 1]
     * 
     * @param u1 lower, proximal
     * @param u2 upper, distal
     */
    public void set(double u1, double u2) {
        m_lowerArmMotor.setDutyCycle(u1);
        m_upperArmMotor.setDutyCycle(u2);
    }

    public void close() {
        m_lowerArmMotor.close();
        m_upperArmMotor.close();
        m_lowerArmEncoder.close();
        m_upperArmEncoder.close();
    }

    @Override
    public String getGlassName() {
        return "Arm";
    }

    @Override
    public void periodic100(double dt) {
        m_viz.viz();
    }
}
