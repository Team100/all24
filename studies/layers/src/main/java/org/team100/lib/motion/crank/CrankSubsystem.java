package org.team100.lib.motion.crank;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** This is an example subsystem using the 1d components. */
public class CrankSubsystem extends Subsystem {
    private final Supplier<Consumer<CrankActuation>> m_actuator;
    private final Supplier<Supplier<CrankActuation>> m_actuation;

    /** The consumer periodically consumers the supplied actuation. */
    public CrankSubsystem(
            Supplier<Supplier<CrankActuation>> actuation,
            Supplier<Consumer<CrankActuation>> actuator) {
        if (actuation == null)
            throw new IllegalArgumentException("null follower");
        m_actuation = actuation;
        m_actuator = actuator;
    }

    @Override
    public void periodic() {
        m_actuator.get().accept(m_actuation.get().get());
    }
}
