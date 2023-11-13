package org.team100.persistent_parameter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * the tunable parameter has four influences:
 * 1. the default
 * 2. the store
 * 3. the console
 * 4. the network
 * 
 * the logic is to use the store, if it exists,
 * and the default if not. the console provides
 * an offset, but the sum also needs to be written
 * to the store. the network can also provide updates.
 * 
 * so essentially we need to track
 * 
 */
public class ExampleSubsystem extends Subsystem {
    private static final double kDefaultA = 1.0;
    private static final double kDefaultB = 2.0;
    private static final String kKeyA = "foo";
    private static final String kKeyB = "bar";
    private final ParameterFactory m_parameters;
    private final PersistentParameter m_parameterA;
    private final PersistentParameter m_parameterB;

    /** @param knob console adjustment, absolute but starting at zero. */
    public ExampleSubsystem(ParameterFactory parameters) {
        m_parameters = parameters;
        m_parameterA = m_parameters.get(kKeyA, kDefaultA);
        m_parameterB = m_parameters.get(kKeyB, kDefaultB);
    }

    @Override
    public void periodic() {
        System.out.printf("%5.3f %5.3f\n",
                m_parameterA.getAsDouble(), m_parameterB.getAsDouble());
    }

    /** Only works when enabled. */
    public Command resetA() {
        return runOnce(m_parameterA::reset);
    }

    public Command resetB() {
        return runOnce(m_parameterB::reset);
    }
}
