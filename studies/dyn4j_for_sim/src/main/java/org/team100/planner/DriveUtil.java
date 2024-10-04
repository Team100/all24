package org.team100.planner;

import org.dyn4j.geometry.Vector2;
import org.team100.commands.Tactics;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveUtil {
    private static final int kAngularP = 5;
    private static final int kCartesianP = 5;

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final boolean m_debug;

    public DriveUtil(SwerveKinodynamics swerveKinodynamics, boolean debug) {
        m_swerveKinodynamics = swerveKinodynamics;
        m_debug = debug;
    }

    /** Proportional feedback with a limiter. */
    public FieldRelativeVelocity goToGoal(Pose2d pose, Pose2d m_goal) {
        if (m_debug)
            System.out.printf(" pose (%5.2f, %5.2f) target (%5.2f, %5.2f)",
                    pose.getX(), pose.getY(), m_goal.getX(), m_goal.getY());
        FieldRelativeDelta transform = FieldRelativeDelta.delta(pose, m_goal);
        Vector2 positionError = new Vector2(transform.getX(), transform.getY());
        double rotationError = MathUtil.angleModulus(transform.getRotation().getRadians());
        Vector2 cartesianU_FB = positionError.product(kCartesianP);
        double angularU_FB = rotationError * kAngularP;
        return new FieldRelativeVelocity(cartesianU_FB.x, cartesianU_FB.y, angularU_FB)
                .clamp(m_swerveKinodynamics.getMaxDriveVelocityM_S(), m_swerveKinodynamics.getMaxAngleSpeedRad_S());
    }

    public FieldRelativeVelocity goToGoalAligned(
            Tactics m_tactics,
            double angleToleranceRad,
            Pose2d pose,
            Translation2d targetFieldRelative) {

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

        boolean aligned = Math.abs(angleError) < angleToleranceRad;

        Translation2d cartesianU_FB = getCartesianError(
                robotToTargetFieldRelative,
                aligned,
                m_debug).times(kCartesianP);

        double angleU_FB = angleError * kAngularP;

        // we also want to turn the intake towards the note
        FieldRelativeVelocity desired = new FieldRelativeVelocity(cartesianU_FB.getX(), cartesianU_FB.getY(), angleU_FB)
                .clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
        if (!aligned) {
            // need to turn? avoid the edges.
            return m_tactics.finish(desired);
        }
        if (robotToTargetFieldRelative.getNorm() < 1) {
            // if close and aligned, don't need tactics at all.
            return desired;
        }
        return m_tactics.finish(desired);
    }

    // this was originally in IndexerSubsystem because it's about the alignment
    // tolerance
    // of the indexer, but i (maybe temporarily?) put it here in order to separate
    // it from the simulator ("RobotBody" etc) stuff
    public static Translation2d getCartesianError(
            Translation2d robotToTargetFieldRelative,
            boolean aligned,
            boolean debug) {
        // Go this far from the note until rotated correctly.
        final double kPickRadius = 1;
        // Correct center-to-center distance for picking.
        final double kMinPickDistanceM = 0.437;
        double distance = robotToTargetFieldRelative.getNorm();
        if (distance < kMinPickDistanceM || !aligned) {
            // target distance is lower than the tangent point: we ran the note
            // over without picking it, so back up.
            // also back up if not aligned.
            if (debug)
                System.out.print(" unaligned");
            double targetDistance = distance - kPickRadius;
            return robotToTargetFieldRelative.times(targetDistance);
        }
        if (debug)
            System.out.print(" aligned");

        // aligned, drive over the note
        return robotToTargetFieldRelative;
    }

}
