package org.team100.lib.encoder.drive;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.units.Distance;

public class MockDriveEncoder implements Encoder100<Distance> {

    @Override
    public double getRate() {
        return 0;
    }

    @Override
    public double getPosition() {
        return 0;
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
