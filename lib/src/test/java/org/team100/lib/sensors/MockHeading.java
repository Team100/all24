package org.team100.lib.sensors;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;

public class MockHeading implements HeadingInterface {
    public Rotation2d rotation = GeometryUtil.kRotationZero;
    public double rate = 0;

    @Override
    public Rotation2d getHeadingNWU() {
        return rotation;
    }

    @Override
    public double getHeadingRateNWU() {
        return rate;
    }
}