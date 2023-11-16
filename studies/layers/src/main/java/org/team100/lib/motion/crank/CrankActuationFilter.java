package org.team100.lib.motion.crank;

import java.util.function.Supplier;

/** Impose actuation limits, e.g. for "soft stops" or velocity limits etc. */
public class CrankActuationFilter implements Supplier<CrankActuation> {

    private static final double positionMin = 0;
    private static final double positionMax = 1;

    private final Supplier<Supplier<CrankActuation>> m_supplier;
    private final Supplier<CrankConfiguration> m_measurement;

    /**
     * Supply the given actuation when the configuration is within limits, otherwise
     * zero actuation.
     */
    public CrankActuationFilter(Supplier<Supplier<CrankActuation>> supplier, Supplier<CrankConfiguration> measurement) {
        m_supplier = supplier;
        m_measurement = measurement;
    }

    @Override
    public CrankActuation get() {
        double position = m_measurement.get().getCrankAngleRad();
        if (position < positionMin || position > positionMax)
            return new CrankActuation(0.0);
        return m_supplier.get().get();
    }
}
