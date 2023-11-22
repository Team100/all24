package org.team100.lib.commands;

import org.team100.lib.motion.drivetrain.HeadingInterface;

import edu.wpi.first.math.geometry.Rotation2d;

public class MockHeading implements HeadingInterface {
    @Override
    public Rotation2d getHeadingNWU() {
        return new Rotation2d();
    }

    @Override
    public double getHeadingRateNWU() {
        return 0;
    }
}