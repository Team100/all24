package org.team100.lib.swerve;

import java.util.function.DoubleBinaryOperator;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveUtil {
    private static final Telemetry t = Telemetry.get();

    /**
     * Check if it would be faster to go to the opposite of the goal heading (and
     * reverse drive direction).
     *
     * @param prevToGoal The rotation from the previous state to the goal state
     *                   (i.e. prev.inverse().rotateBy(goal)).
     * @return True if the shortest path to achieve this rotation involves flipping
     *         the drive direction.
     */
    public static boolean flipHeading(Rotation2d prevToGoal) {
        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
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
            double max_deviation,
            int max_iterations) {
        f_1 = SwerveUtil.unwrapAngle(f_0, f_1);

        double diff = f_1 - f_0;

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

    /**
     * f is speed: hypot(x,y)
     */
    public static double findDriveMaxS(
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double max_vel_step,
            int max_iterations) {
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
    public static boolean makeStop(
            ChassisSpeeds desiredState,
            SwerveModuleState[] desiredModuleStates,
            SwerveModuleState[] prevModuleStates) {
        boolean need_to_steer = true;
        if (GeometryUtil.toTwist2d(desiredState).equals(GeometryUtil.kTwist2dIdentity)) {
            need_to_steer = false;
            for (int i = 0; i < prevModuleStates.length; ++i) {
                desiredModuleStates[i].angle = prevModuleStates[i].angle;
                desiredModuleStates[i].speedMetersPerSecond = 0.0;
            }
        }
        return need_to_steer;
    }

    /**
     * Compares magnitudes of the current and final state.
     * For transitions that involve both speed and angle changes,
     * the correct velocity profile often involves slowing and then
     * speeding up, but this method doesn't capture that.
     */
    public static double getMaxVelStep(
            SwerveKinodynamics m_limits,
            double prev_vx_i,
            double prev_vy_i,
            double desired_vx_i,
            double desired_vy_i,
            double kDtSec) {
        boolean isAccel = getIsAccel(prev_vx_i, prev_vy_i, desired_vx_i, desired_vy_i);

        double max_vel_step = isAccel ? kDtSec * m_limits.getMaxDriveAccelerationM_S2()
                : kDtSec * m_limits.getMaxDriveDecelerationM_S2();
        t.log(Level.DEBUG, "/setpoint_generator/max_vel_step", max_vel_step);
        return max_vel_step;
    }

    static boolean getIsAccel(
            double prev_vx_i,
            double prev_vy_i,
            double desired_vx_i,
            double desired_vy_i) {
        double prevV = Math.hypot(prev_vx_i, prev_vy_i);
        t.log(Level.DEBUG, "/setpoint_generator/prevV", prevV);

        double desiredV = Math.hypot(desired_vx_i, desired_vy_i);
        t.log(Level.DEBUG, "/setpoint_generator/desiredV", desiredV);

        return desiredV >= prevV;
    }

    /**
     * This method projects the line onto the prev state.
     */
    public static double getMaxVelStep2(
            SwerveKinodynamics m_limits,
            double prev_vx_i,
            double prev_vy_i,
            double desired_vx_i,
            double desired_vy_i,
            double kDtSec) {

        boolean isAccel = getIsAccel2(
                prev_vx_i,
                prev_vy_i,
                desired_vx_i,
                desired_vy_i);

        double max_vel_step = isAccel ? kDtSec * m_limits.getMaxDriveAccelerationM_S2()
                : kDtSec * m_limits.getMaxDriveDecelerationM_S2();
        t.log(Level.DEBUG, "/setpoint_generator/max_vel_step", max_vel_step);
        return max_vel_step;
    }

    static boolean getIsAccel2(
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
