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
    private DoublePredicate m_enabler;

    public CrankSubsystem(Supplier<Supplier<CrankActuation>> actuation, Supplier<Consumer<CrankActuation>> actuator) {
        if (actuation == null)
            throw new IllegalArgumentException("null follower");
        m_actuation = actuation;
        m_actuator = actuator;
        m_actuationFilter = x -> x;
        m_enabler = x -> true;
    }

    public void setActuationFilter(UnaryOperator<CrankActuation> filter) {
        m_actuationFilter = filter;
    }

    public void setEnable(DoublePredicate enabler) {
        if (enabler == null)
            throw new IllegalArgumentException("null enabler");
        m_enabler = enabler;
    }

    private boolean enabled() {
        if (m_enabler == null)
            return false;
        double measurement = 0.0; // TODO: use a real measurement here.
        return m_enabler.test(measurement);
    }

    @Override
    public void periodic() {
        if (!enabled()) {
            m_actuator.get().accept(new CrankActuation(0));
            return;
        }

        CrankActuation actuation = m_actuation.get().get();

        if (m_actuationFilter != null) {
            actuation = m_actuationFilter.apply(actuation);
        }

        m_actuator.get().accept(actuation);
    }

}
