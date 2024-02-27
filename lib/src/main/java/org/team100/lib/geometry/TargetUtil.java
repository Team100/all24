package org.team100.lib.geometry;

import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Static methods for targeting.
 */
public class TargetUtil {

    /**
     * Absolute bearing to the target.
     * 
     * The bearing is only a valid shooting solution if both the robot and the
     * target are at rest!
     * 
     * If the robot and/or target is moving, then the shooting solution needs to
     * lead or lag the target.
     * 
     * @param robot field-relative robot translation
     * @param target field-relative target translation
     * @return absolute bearing from robot to target
     */
    public static Rotation2d bearing(Translation2d robot, Translation2d target) {
        return target.minus(robot).getAngle();
    }

    /**
     * Apparent motion of the target, NWU rad/s.
     * 
     * The theta profile goal is to move at this rate, i.e. tracking the apparent
     * movement.
     * 
     * @param state  current robot state4
     * @param target field-relative target position
     * @return apparent rotation of the target around the robot, radians per second
     */
    public static double targetMotion(SwerveState state, Translation2d target) {
        Translation2d robot = state.pose().getTranslation();
        Translation2d translation = target.minus(robot);
        double range = translation.getNorm();
        Rotation2d bearing = translation.getAngle();
        Twist2d twist = state.twist();
        Rotation2d course = new Rotation2d(twist.dx, twist.dy);
        Rotation2d relativeBearing = bearing.minus(course);
        double speed = GeometryUtil.norm(twist);
        return speed * relativeBearing.getSin() / range;
    }

    private TargetUtil() {
        //
    }
}
