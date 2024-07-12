package org.team100.lib.geometry;

import java.util.Optional;

import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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
     * @param robot  field-relative robot translation
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
     * @param state  current robot state
     * @param target field-relative target position
     * @return apparent rotation of the target around the robot, radians per second
     */
    public static double targetMotion(SwerveState state, Translation2d target) {
        Translation2d robot = state.pose().getTranslation();
        Translation2d translation = target.minus(robot);

        Rotation2d bearing = translation.getAngle();
        FieldRelativeVelocity twist = state.velocity();
        Rotation2d relativeBearing = getRelativeBearing(bearing, twist);

        double speed = twist.norm();
        double range = translation.getNorm();
        return speed * relativeBearing.getSin() / range;
    }

    private static Rotation2d getRelativeBearing(Rotation2d bearing, FieldRelativeVelocity velo) {
        Optional<Rotation2d> course = velo.angle();
        if (course.isEmpty()) {
            // if we're not moving, the relative bearing is just the bearing.
            return bearing;
        }
        return bearing.minus(course.get());
    }

    private TargetUtil() {
        //
    }
}
