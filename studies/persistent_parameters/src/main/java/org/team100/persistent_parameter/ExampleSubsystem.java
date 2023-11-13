package org.team100.persistent_parameter;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ExampleSubsystem extends Subsystem {
    private final PersistentParameter m_parameterA;
    private final PersistentParameter m_parameterB;
    private final PersistentParameter m_parameterC;

    public ExampleSubsystem(ParameterFactory parameters) {
        m_parameterA = parameters.get("foo", 1.0);
        // mapped to a different knob
        m_parameterB = parameters.get("bar", 2.0);
        // this parameter gets NT updates but not knob updates
        m_parameterC = parameters.get("no_knob", 20.0);
    }

    @Override
    public void periodic() {
        System.out.printf("%5.2f %5.2f %5.2f\n",
                m_parameterA.getAsDouble(),
                m_parameterB.getAsDouble(),
                m_parameterC.getAsDouble());
    }

    public void doNothing() {
        //
    }
}
