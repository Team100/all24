package org.team100.lib.encoder.turning;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.units.Measure100;

public class MockEncoder100<T extends Measure100> implements Encoder100<T> {
    public double angle = 0;
    public double rate = 0;
    @Override
    public Double getPosition() {
        return angle;
    }

    @Override
    public double getRate() {
        return rate;
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
