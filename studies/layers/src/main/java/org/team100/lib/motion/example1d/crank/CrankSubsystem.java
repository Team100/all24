package org.team100.lib.motion.example1d.crank;

import java.util.function.Consumer;
import java.util.function.DoublePredicate;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** This is an example subsystem using the 1d components. */
public class CrankSubsystem extends Subsystem {

    private final Supplier<Consumer<CrankActuation>> m_actuator;
    private final Supplier<Supplier<CrankActuation>> m_actuation;
    private UnaryOperator<CrankActuation> m_actuationFilter;

    public CrankSubsystem(Supplier<Supplier<CrankActuation>> actuation, Supplier<Consumer<CrankActuation>> actuator) {
        if (actuation == null)
            throw new IllegalArgumentException("null follower");
        m_actuation = actuation;
        m_actuator = actuator;
        m_actuationFilter = x -> x;
    }

    public void setActuationFilter(UnaryOperator<CrankActuation> filter) {
        m_actuationFilter = filter;
    }


    @Override
    public void periodic() {
        CrankActuation actuation = m_actuation.get().get();

        if (m_actuationFilter != null) {
            actuation = m_actuationFilter.apply(actuation);
        }

        m_actuator.get().accept(actuation);
    }

}
