package org.team100.lib.encoder.turning;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.units.Angle;

public class MockTurningEncoder implements Encoder100<Angle> {
    public double angle = 0;
    public double rate = 0;
    @Override
    public double getPosition() {
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
