package org.team100.lib.encoder;

import java.util.OptionalDouble;

public class MockRotaryPositionSensor implements RotaryPositionSensor {
    public double angle = 0;
    public double rate = 0;

    @Override
    public OptionalDouble getPositionRad() {
        return OptionalDouble.of(angle);
    }

    @Override
    public OptionalDouble getRateRad_S() {
        return OptionalDouble.of(rate);
    }

    @Override
    public void close() {
        //
    }

}
