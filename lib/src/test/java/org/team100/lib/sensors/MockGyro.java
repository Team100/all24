package org.team100.lib.sensors;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;

public class MockGyro implements Gyro {
    public Rotation2d rotation = GeometryUtil.kRotationZero;
    public double rate = 0;

    @Override
    public Rotation2d getYawNWU() {
        return rotation;
    }

    @Override
    public double getYawRateNWU() {
        return rate;
    }

    @Override
    public Rotation2d getPitchNWU() {
        return GeometryUtil.kRotationZero;
    }

    @Override
    public Rotation2d getRollNWU() {
        return GeometryUtil.kRotationZero;
    }

    @Override
    public void periodic() {
        //
    }
}