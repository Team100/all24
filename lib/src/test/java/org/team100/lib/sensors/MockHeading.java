package org.team100.lib.sensors;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;

public class MockHeading implements HeadingInterface {
    @Override
    public Rotation2d getHeadingNWU() {
        return GeometryUtil.kRotationZero;
    }

    @Override
    public double getHeadingRateNWU() {
        return 0;
    }
}