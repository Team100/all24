package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Manual cartesian control, with rotational control based on a target position.
 * 
 * This is useful for shooting solutions, or for keeping the camera pointed at
 * something.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 * 
 * TODO: add velocity feedback.
 * TODO: replace the PID controller with multiplication
 * TODO: lead the target based on velocity.
 */
public class ManualWithTargetLock {
    private final Telemetry t = Telemetry.get();
    private final Supplier<Translation2d> m_target;
    private final PIDController m_thetaController;

    public ManualWithTargetLock(
            Supplier<Translation2d> target,
            PIDController thetaController) {
        m_target = target;
        m_thetaController = thetaController;
    }

    public Twist2d apply(Pose2d currentPose, Twist2d twist1_1) {
        Translation2d currentTranslation = currentPose.getTranslation();
        Rotation2d angleToTarget = fieldRelativeAngleToTarget(
                currentTranslation, m_target.get());
        Rotation2d currentAngle = currentPose.getRotation();

        // MotionState goal = new MotionState(MathUtil.angleModulus(latchedPov.getRadians()), 0);




        return null;

    }

    static Rotation2d fieldRelativeAngleToTarget(
            Translation2d robot,
            Translation2d target) {
        return target.minus(robot).getAngle();
    }

}
