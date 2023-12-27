package org.team100.lib.sensors;

import org.team100.lib.config.Identity;
import org.team100.lib.motion.drivetrain.SwerveModuleCollection;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * Produces real or simulated gyros depending on identity.
 */
public class HeadingFactory {

    public static HeadingInterface get(
            SwerveDriveKinematics kinematics,
            SwerveModuleCollection collection) {
        switch (Identity.instance) {
            case BLANK:
                // for simulation
                return new SimulatedHeading(kinematics, collection);
            default:
                RedundantGyroInterface ahrsclass = new RedundantGyroFactory(Identity.instance).get();
                return new Heading(ahrsclass);
        }
    }
}
