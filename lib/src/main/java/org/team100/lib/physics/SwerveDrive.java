package org.team100.lib.physics;

import java.text.DecimalFormat;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// TODO remove this class
public class SwerveDrive {
    // All units must be SI!

    // Self-explanatory.  Measure by rolling the robot a known distance and counting encoder ticks.
    protected final double wheel_radius_;  // m

    // "Effective" kinematic wheelbase radius.  Might be larger than theoretical to compensate for skid steer.  Measure
    // by turning the robot in place several times and figuring out what the equivalent wheelbase radius is.
    protected final double effective_wheelbase_radius_;  // m

    public SwerveDrive(final double wheel_radius,
                       final double effective_wheelbase_radius) {
        wheel_radius_ = wheel_radius;
        effective_wheelbase_radius_ = effective_wheelbase_radius;
    }

    // Can refer to velocity or acceleration depending on context.
    public static class ChassisState {
        public Translation2d movement;
        public Rotation2d heading;

        public ChassisState(Translation2d movement, Rotation2d heading) {
            this.heading = heading;
            this.movement = movement;
        }

        public ChassisState(Translation2d movement) {
            this.movement = movement;
            this.heading = GeometryUtil.kRotationIdentity;
        }


        public ChassisState() {
            this.movement = GeometryUtil.kTranslation2dIdentity;
            this.heading = GeometryUtil.kRotationIdentity;
        }

        @Override
        public String toString() {
            DecimalFormat fmt = new DecimalFormat("#0.000");
            return fmt.format(movement.getNorm())/* + ", " + fmt.format(heading.getRadians())*/;
        }
    }
}