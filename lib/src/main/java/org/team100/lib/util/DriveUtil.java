package org.team100.lib.util;

import java.util.Optional;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleDeltas;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePosition100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePositions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;

public class DriveUtil {
    /**
     * Scales driver input to field-relative velocity.
     * 
     * This makes no attempt to address infeasibilty, it just multiplies.
     * 
     * @param twist    [-1,1]
     * @param maxSpeed meters per second
     * @param maxRot   radians per second
     * @return meters and rad per second as specified by speed limits
     */
    public static FieldRelativeVelocity scale(DriverControl.Velocity twist, double maxSpeed, double maxRot) {
        return new FieldRelativeVelocity(
                maxSpeed * MathUtil.clamp(twist.x(), -1, 1),
                maxSpeed * MathUtil.clamp(twist.y(), -1, 1),
                maxRot * MathUtil.clamp(twist.theta(), -1, 1));
    }

    /**
     * Scales driver input to robot-relative velocity.
     * 
     * This makes no attempt to address infeasibilty, it just multiplies.
     * 
     * @param twist    [-1,1]
     * @param maxSpeed meters per second
     * @param maxRot   radians per second
     * @return meters and rad per second as specified by speed limits
     */
    public static ChassisSpeeds scaleChassisSpeeds(DriverControl.Velocity twist, double maxSpeed, double maxRot) {
        return new ChassisSpeeds(
                maxSpeed * MathUtil.clamp(twist.x(), -1, 1),
                maxSpeed * MathUtil.clamp(twist.y(), -1, 1),
                maxRot * MathUtil.clamp(twist.theta(), -1, 1));
    }

    /**
     * Clamp the translational velocity to the unit circle by clipping outside.
     * 
     * This means that there will be no stick response outside the circle, but the
     * inside will be unchanged.
     * 
     * The argument for clipping is that it leaves the response inside the circle
     * alone: with squashing, the diagonals are more sensitive.
     * 
     * The argument for squashing is that it preserves relative response in
     * the corners: with clipping, going full speed diagonally and then "slowing
     * down a little" will do nothing.
     * 
     * If you'd like to avoid clipping, then squash the input upstream, in the
     * control class.
     */
    public static DriverControl.Velocity clampTwist(DriverControl.Velocity input, double maxMagnitude) {
        double hyp = Math.hypot(input.x(), input.y());
        if (hyp < 1e-12)
            return input;
        double clamped = Math.min(hyp, maxMagnitude);
        double ratio = clamped / hyp;
        return new DriverControl.Velocity(ratio * input.x(), ratio * input.y(), input.theta());
    }

    /** crash the robot in simulation or just substitute zero in prod */
    public static void checkSpeeds(ChassisSpeeds speeds) {
        try {
            if (Double.isNaN(speeds.vxMetersPerSecond))
                throw new IllegalStateException("vx is NaN");
            if (Double.isNaN(speeds.vyMetersPerSecond))
                throw new IllegalStateException("vy is NaN");
            if (Double.isNaN(speeds.omegaRadiansPerSecond))
                throw new IllegalStateException("omega is NaN");
        } catch (IllegalStateException e) {
            if (RobotBase.isReal()) {
                Util.warn("NaN speeds!");
                speeds.vxMetersPerSecond = 0;
                speeds.vyMetersPerSecond = 0;
                speeds.omegaRadiansPerSecond = 0;
                throw e;
                // return;
            }
            // in test/sim, it's ok to throw
            throw e;
        }
    }

    public static void checkTwist(Twist2d twist) {
        try {
            if (Double.isNaN(twist.dx))
                throw new IllegalStateException("dx is Nan");
            if (Double.isNaN(twist.dy))
                throw new IllegalStateException("dy is Nan");
            if (Double.isNaN(twist.dtheta))
                throw new IllegalStateException("dtheta is Nan");
        } catch (IllegalStateException e) {
            throw e;
        }
    }

    /**
     * The inverse kinematics wants this to represent a geodesic, which
     * means that the steering doesn't change between start and end.
     */
    public static SwerveModuleDeltas modulePositionDelta(
            SwerveModulePositions start,
            SwerveModulePositions end) {
        return new SwerveModuleDeltas(
                delta(start.frontLeft(), end.frontLeft()),
                delta(start.frontRight(), end.frontRight()),
                delta(start.rearLeft(), end.rearLeft()),
                delta(start.rearRight(), end.rearRight()));
    }

    public static SwerveModulePositions modulePositionFromDelta(
            SwerveModulePositions initial,
            SwerveModuleDeltas delta) {
        return new SwerveModulePositions(
                plus(initial.frontLeft(), delta.frontLeft()),
                plus(initial.frontRight(), delta.frontRight()),
                plus(initial.rearLeft(), delta.rearLeft()),
                plus(initial.rearRight(), delta.rearRight()));
    }

    /**
     * Delta for one module, straight line path using the end angle.
     */
    public static SwerveModuleDelta delta(
            SwerveModulePosition100 start,
            SwerveModulePosition100 end) {
        double deltaM = end.distanceMeters - start.distanceMeters;
        if (end.angle.isPresent()) {
            return new SwerveModuleDelta(deltaM, end.angle);
        }
        // the angle might be empty, if the encoder has failed
        // (which can seem to happen if the robot is *severely* overrunning).
        return new SwerveModuleDelta(0, Optional.empty());
    }

    public static SwerveModulePosition100 plus(
            SwerveModulePosition100 start,
            SwerveModuleDelta delta) {
        double posM = start.distanceMeters + delta.distanceMeters;
        if (delta.angle.isPresent()) {
            return new SwerveModulePosition100(posM, delta.angle);
        }
        // if there's no delta angle, we're not going anywhere.
        return start;
    }

    private DriveUtil() {
    }
}
