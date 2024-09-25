package org.team100.lib.swerve;

import java.util.function.DoubleBinaryOperator;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveUtil {
    /**
     * Check if it would be faster to go to the opposite of the goal heading (and
     * reverse drive direction).
     *
     * @param prevToGoal The rotation from the previous state to the goal state
     * @return True if the shortest path to achieve this rotation involves flipping
     *         the drive direction.
     */
    public static boolean shouldFlip(Rotation2d prevToGoal) {
        return Math.abs(MathUtil.angleModulus(prevToGoal.getRadians())) > Math.PI / 2.0;
    }

    public static double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) {
            return angle - 2.0 * Math.PI;
        } else if (diff < -Math.PI) {
            return angle + 2.0 * Math.PI;
        } else {
            return angle;
        }
    }

    /**
     * 
     * @param x_0            previous vx
     * @param y_0            previoux vy
     * @param f_0            previous steering angle
     * @param x_1            desired vx
     * @param y_1            desired vy
     * @param f_1            desired steering angle
     * @param max_deviation  max angle step
     * @param max_iterations
     * @return
     */
    public static double findSteeringMaxS(
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double f_2,
            double max_deviation,
            int max_iterations) {
        f_1 = SwerveUtil.unwrapAngle(f_0, f_1);

        if (Math.abs(f_2) <= max_deviation) {
            // Can go all the way to s=1.
            return 1.0;
        }

        double offset = f_0 + Math.signum(f_2) * max_deviation;

        DoubleBinaryOperator func = (x, y) -> SwerveUtil.unwrapAngle(f_0, Math.atan2(y, x)) - offset;

        return Math100.findRoot(
                func,
                x_0, y_0, f_0 - offset,
                x_1, y_1, f_1 - offset,
                max_iterations);
    }

    /**
     * 
     * @param x_0            previous vx
     * @param y_0            previoux vy
     * @param f_0            previous steering angle
     * @param x_1            desired vx
     * @param y_1            desired vy
     * @param f_1            desired steering angle
     * @param max_deviation  max angle step
     * @param max_iterations
     * @return
     */
    public static double findSteeringMaxS(
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double max_deviation,
            int max_iterations) {
        f_1 = SwerveUtil.unwrapAngle(f_0, f_1);

        double diff = f_1-f_0;
        
        if (Math.abs(diff) <= max_deviation) {
            // Can go all the way to s=1.
            return 1.0;
        }

        double offset = f_0 + Math.signum(diff) * max_deviation;

        DoubleBinaryOperator func = (x, y) -> SwerveUtil.unwrapAngle(f_0, Math.atan2(y, x)) - offset;

        return Math100.findRoot(
                func,
                x_0, y_0, f_0 - offset,
                x_1, y_1, f_1 - offset,
                max_iterations);
    }

    public static double findDriveMaxS(
            double x_0,
            double y_0,
            double x_1,
            double y_1,
            double max_vel_step,
            int max_iterations) {
        double f_0 = Math.hypot(x_0, y_0);
        double f_1 = Math.hypot(x_1, y_1);

        double diff = f_1 - f_0;

        if (Math.abs(diff) <= max_vel_step) {
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_vel_step;
        DoubleBinaryOperator func = (x, y) -> (Math.hypot(x, y) - offset);
        return Math100.findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
    }

    /**
     * DesiredState is a complete stop. In this case, module angle is
     * arbitrary, so just use the previous angle.
     */
    public static boolean desiredIsStopped(
            ChassisSpeeds desiredState,
            SwerveModuleState100[] desiredModuleStates,
            SwerveModuleState100[] prevModuleStates) {
        if (GeometryUtil.isZero(desiredState)) {
            for (int i = 0; i < prevModuleStates.length; ++i) {
                desiredModuleStates[i].angle = prevModuleStates[i].angle;
                desiredModuleStates[i].speedMetersPerSecond = 0.0;
            }
            return true;
        }
        return false;
    }

    /**
     * Find the desired dv. Project it on to the previous v: if the projection is
     * positive, we're accelerating, so use the accel limit to find the maximum
     * allowed dv for the supplied dt. Otherwise use the decel limit.
     */
    public static double getMaxVelStep(
            SwerveKinodynamics m_limits,
            double prev_vx,
            double prev_vy,
            double desired_vx,
            double desired_vy) {
        return TimedRobot100.LOOP_PERIOD_S * getAccelLimit(
                m_limits,
                prev_vx,
                prev_vy,
                desired_vx,
                desired_vy);
    }

    /**
     * At low speed, accel is limited by the current limiters.
     * At high speed, accel is limited by back EMF.
     * Deceleration limits are different: back EMF is helping in that case.
     * 
     * @see SwerveDriveDynamicsConstraint.getMinMaxAcceleration().
     */
    public static double getAccelLimit(
            SwerveKinodynamics m_limits,
            double prev_vx,
            double prev_vy,
            double desired_vx,
            double desired_vy) {
        if (isAccel(prev_vx, prev_vy, desired_vx, desired_vy)) {
            double speedM_S = Math.hypot(prev_vx, prev_vy);
            return minAccel(m_limits, speedM_S);
        }
        return m_limits.getMaxDriveDecelerationM_S2();
    }

    public static double minAccel(SwerveKinodynamics m_limits, double velocity) {
        double speedFraction = Math100.limit(velocity / m_limits.getMaxDriveVelocityM_S(), 0, 1);
        double backEmfLimit = 1 - speedFraction;
        double backEmfLimitedAcceleration = backEmfLimit * m_limits.getStallAccelerationM_S2();
        double currentLimitedAcceleration = m_limits.getMaxDriveAccelerationM_S2();
        return Math.min(backEmfLimitedAcceleration, currentLimitedAcceleration);
    }

    /**
     * Find the desired dv. Project it on to the previous v: if the projection is
     * positive, we're accelerating, otherwise decelerating.
     * 
     * This correctly captures sharp turns as decelerations; simply comparing the
     * magnitudes of initial and final velocities is not correct.
     */
    static boolean isAccel(
            double prev_vx_i,
            double prev_vy_i,
            double desired_vx_i,
            double desired_vy_i) {
        double lx = desired_vx_i - prev_vx_i;
        double ly = desired_vy_i - prev_vy_i;
        double dot = prev_vx_i * lx + prev_vy_i * ly;
        return (dot >= 0);
    }

    private SwerveUtil() {
        //
    }
}
