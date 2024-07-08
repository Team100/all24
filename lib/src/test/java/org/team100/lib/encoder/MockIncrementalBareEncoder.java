package org.team100.lib.encoder;

import java.util.OptionalDouble;

public class MockIncrementalBareEncoder implements IncrementalBareEncoder {
    public double position = 0;
    public double velocity = 0;

    @Override
    public OptionalDouble getVelocityRad_S() {
        return OptionalDouble.of(velocity);
    }

    @Override
    public OptionalDouble getPositionRad() {
        return OptionalDouble.of(position);
    }

    @Override
    public void reset() {
        //
    }

    @Override
    public void close() {
        //
    }
    
}
