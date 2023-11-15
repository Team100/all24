package org.team100.persistent_parameter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ExampleSubsystem extends Subsystem {
    private final DoubleSupplier m_parameterA;
    private final DoubleSupplier m_parameterB;
    private final DoubleSupplier m_parameterC;
    private final DoubleSupplier m_parameterD;

    public ExampleSubsystem(ParameterFactory parameters) {
        m_parameterA = parameters.inconstant("foo", 1.0);
        // mapped to a different knob
        m_parameterB = parameters.inconstant("bar", 2.0);
        // this parameter gets NT updates but not knob updates
        m_parameterC = parameters.inconstant("no_knob", 20.0);
        m_parameterD = parameters.constant("/nesting/a_constant", 30.0);
    }

    @Override
    public void periodic() {
        System.out.printf("%5.2f %5.2f %5.2f %5.2f\n",
                m_parameterA.getAsDouble(),
                m_parameterB.getAsDouble(),
                m_parameterC.getAsDouble(),
                m_parameterD.getAsDouble());
    }

    public void doNothing() {
        //
    }
}
