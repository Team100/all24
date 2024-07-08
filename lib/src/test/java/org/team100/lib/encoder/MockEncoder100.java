package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.units.Measure100;

public class MockEncoder100<T extends Measure100> implements SettableEncoder<T> {
    public double angle = 0;
    public double rate = 0;

    @Override
    public OptionalDouble getPosition() {
        return OptionalDouble.of(angle);
    }

    @Override
    public OptionalDouble getRate() {
        return OptionalDouble.of(rate);
    }

    @Override
    public void reset() {
        //
    }

    @Override
    public void close() {
        //
    }

    @Override
    public void setPosition(double position) {
        this.angle = position;
    }
}
