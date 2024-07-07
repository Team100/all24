package org.team100.lib.encoder;

import java.util.OptionalDouble;

public class MockSettableAngularEncoder implements SettableAngularEncoder {
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
