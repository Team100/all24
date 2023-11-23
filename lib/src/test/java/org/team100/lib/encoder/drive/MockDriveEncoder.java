package org.team100.lib.encoder.drive;

public class MockDriveEncoder implements DriveEncoder {

    @Override
    public double getRate() {
        return 0;
    }

    @Override
    public double getDistance() {
        return 0;
    }

    @Override
    public void reset() {
        //
    }

}
