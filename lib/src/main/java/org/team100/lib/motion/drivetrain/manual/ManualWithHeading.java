package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Function;
import java.util.function.Supplier;

import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

/** Extracts logic from DriveWithHeading. */
public class ManualWithHeading implements Function<Twist2d, Twist2d> {
    private final Telemetry t = Telemetry.get();
    private final Supplier<Rotation2d> m_desiredRotation;

    public ManualWithHeading(Supplier<Rotation2d> desiredRotation) {
        m_desiredRotation = desiredRotation;
    }

    @Override
    public Twist2d apply(Twist2d arg0) {

        return null;
    }

}
