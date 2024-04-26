package org.team100.lib.motion.drivetrain.kinodynamics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * This is just a container for the field-relative difference between two poses.
 * 
 * Everywhere I used pose.minus() I think it's wrong.
 */
public class FieldRelativeDelta {
    private final Translation2d m_translation;
    private final Rotation2d m_rotation;

    public FieldRelativeDelta(Translation2d translation, Rotation2d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    /** Return a delta from start to end. */
    public static FieldRelativeDelta delta(Pose2d start, Pose2d end) {
        Translation2d t = end.getTranslation().minus(start.getTranslation());
        Rotation2d r = end.getRotation().minus(start.getRotation());
        return new FieldRelativeDelta(t, r);
    }

    public FieldRelativeDelta limit(double cartesian, double rotation) {
        return new FieldRelativeDelta(
                new Translation2d(
                        Math.min(cartesian, m_translation.getX()),
                        Math.min(cartesian, m_translation.getY())),
                new Rotation2d(
                        Math.min(rotation, m_rotation.getRadians())));
    }

    public FieldRelativeDelta times(double scalar) {
        return new FieldRelativeDelta(m_translation.times(scalar), m_rotation.times(scalar));
    }

    public FieldRelativeDelta div(double scalar) {
        return new FieldRelativeDelta(m_translation.div(scalar), m_rotation.div(scalar));
    }

    public double getX() {
        return m_translation.getX();
    }

    public double getY() {
        return m_translation.getY();
    }

    public Translation2d getTranslation() {
        return m_translation;
    }

    public Rotation2d getRotation() {
        return m_rotation;
    }

}
