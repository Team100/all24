package org.team100.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.RobotBase;

public class DriveUtil {
    /**
     * This makes no attempt to address infeasibilty, it just multiplies.
     * 
     * @param twist    [-1,1]
     * @param maxSpeed meters per second
     * @param maxRot   radians per second
     * @return meters and rad per second as specified by speed limits
     */
    public static Twist2d scale(Twist2d twist, double maxSpeed, double maxRot) {
        return new Twist2d(
                maxSpeed * MathUtil.clamp(twist.dx, -1, 1),
                maxSpeed * MathUtil.clamp(twist.dy, -1, 1),
                maxRot * MathUtil.clamp(twist.dtheta, -1, 1));
    }

    /**
     * This makes no attempt to address infeasibilty, it just multiplies.
     * 
     * @param twist    [-1,1]
     * @param maxSpeed meters per second
     * @param maxRot   radians per second
     * @return meters and rad per second as specified by speed limits
     */
    public static ChassisSpeeds scaleChassisSpeeds(Twist2d twist, double maxSpeed, double maxRot) {
        Twist2d scaled = scale(twist, maxSpeed, maxRot);
        return new ChassisSpeeds(
                scaled.dx,
                scaled.dy,
                scaled.dtheta);
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
    public static Twist2d clampTwist(Twist2d input, double maxMagnitude) {
        double hyp = Math.hypot(input.dx, input.dy);
        if (hyp < 1e-12)
            return input;
        double clamped = Math.min(hyp, maxMagnitude);
        double ratio = clamped / hyp;
        return new Twist2d(ratio * input.dx, ratio * input.dy, input.dtheta);
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
     * Path between start and end is assumed to be a circular arc so the
     * angle of the delta is the angle of the chord between the endpoints,
     * i.e. the average angle. This might not be a good assumption if the positional
     * control is at a lower level, so that the motion is not uniform during the
     * control period.
     * 
     * Note the arc is assumed to be the same length as the chord, though, i.e. the
     * angles are assumed to be close to each other.
     */
    public static SwerveModulePosition[] modulePositionDelta(
            SwerveDriveWheelPositions start,
            SwerveDriveWheelPositions end) {
        if (start.positions.length != end.positions.length) {
            throw new IllegalArgumentException("Inconsistent number of modules!");
        }
        SwerveModulePosition[] newPositions = new SwerveModulePosition[start.positions.length];
        for (int i = 0; i < start.positions.length; i++) {
            SwerveModulePosition startModule = start.positions[i];
            SwerveModulePosition endModule = end.positions[i];
            newPositions[i] = new SwerveModulePosition(
                    endModule.distanceMeters - startModule.distanceMeters,
                    endModule.angle.interpolate(startModule.angle, 0.5));
        }
        return newPositions;
    }

    private DriveUtil() {
    }
}
