package org.team100.planner;

import org.dyn4j.geometry.Vector2;
import org.team100.commands.Tactics;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.subsystems.IndexerSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Drive {
    private static final int kAngularP = 5;
    private static final int kCartesianP = 5;

    /** Proportional feedback with a limiter. */
    public static FieldRelativeVelocity goToGoal(Pose2d pose, Pose2d m_goal, boolean debug) {
        if (debug)
            System.out.printf(" pose (%5.2f, %5.2f) target (%5.2f, %5.2f)",
                    pose.getX(), pose.getY(), m_goal.getX(), m_goal.getY());
        FieldRelativeDelta transform = FieldRelativeDelta.delta(pose, m_goal);
        Vector2 positionError = new Vector2(transform.getX(), transform.getY());
        double rotationError = MathUtil.angleModulus(transform.getRotation().getRadians());
        Vector2 cartesianU_FB = positionError.product(kCartesianP);
        double angularU_FB = rotationError * kAngularP;
        return new FieldRelativeVelocity(cartesianU_FB.x, cartesianU_FB.y, angularU_FB)
                .clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
    }

    public static FieldRelativeVelocity goToGoalAligned(
            Tactics m_tactics,
            IndexerSubsystem m_indexer,
            Pose2d pose,
            Translation2d targetFieldRelative,
            boolean m_debug) {

        if (m_debug)
            System.out.printf(" pose (%5.2f, %5.2f) target (%5.2f, %5.2f)",
                    pose.getX(), pose.getY(),
                    targetFieldRelative.getX(), targetFieldRelative.getY());

        Translation2d robotToTargetFieldRelative = targetFieldRelative.minus(pose.getTranslation());
        Rotation2d robotToTargetAngleFieldRelative = robotToTargetFieldRelative.getAngle();
        // intake is on the back
        Rotation2d intakeAngleFieldRelative = GeometryUtil.flip(pose.getRotation());
        double angleError = MathUtil.angleModulus(
                robotToTargetAngleFieldRelative.minus(intakeAngleFieldRelative).getRadians());

        boolean aligned = m_indexer.aligned(angleError);

        Translation2d cartesianU_FB = m_indexer.getCartesianError(
                robotToTargetFieldRelative,
                aligned).times(kCartesianP);

        double angleU_FB = angleError * kAngularP;

        // we also want to turn the intake towards the note
        FieldRelativeVelocity desired = new FieldRelativeVelocity(cartesianU_FB.getX(), cartesianU_FB.getY(), angleU_FB)
                .clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
        // need to turn? avoid the edges.
        return m_tactics.finish(desired, true, !aligned, true);
    }

    private Drive() {
        //
    }
}
