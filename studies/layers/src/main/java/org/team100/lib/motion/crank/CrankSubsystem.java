package org.team100.lib.motion.crank;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** This is an example subsystem using the 1d components. */
public class CrankSubsystem extends Subsystem implements Indicator.Visible {
    private final Supplier<Actuations> m_actuation;
    private final Supplier<Actuator> m_actuator;

    /** The consumer periodically consumers the supplied actuation. */
    public CrankSubsystem(
            Supplier<Actuations> actuation,
            Supplier<Actuator> actuator) {
        if (actuation == null)
            throw new IllegalArgumentException("null actuation");
        if (actuator == null)
            throw new IllegalArgumentException("null actuator");
        m_actuation = actuation;
        m_actuator = actuator;
    }

    @Override
    public void periodic() {
        m_actuator.get().accept(m_actuation.get().get());
    }

    @Override
    public void accept(Indicator indicator) {
        m_actuation.get().accept(indicator);
        m_actuator.get().accept(indicator);
        indicator.indicate(this);
    }
}
