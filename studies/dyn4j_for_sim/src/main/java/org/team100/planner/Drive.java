package org.team100.planner;

import org.dyn4j.geometry.Vector2;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

public class Drive {
    private static final int kAngularP = 10;
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

    private Drive() {
        //
    }
}
