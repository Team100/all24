package org.team100.lib.sensors;

import org.team100.lib.config.Identity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;

/**
 * Produces real or simulated gyros depending on identity.
 */
public class HeadingFactory {

    public static HeadingInterface get(
            SwerveKinodynamics kinodynamics,
            SwerveModuleCollection collection) {
        switch (Identity.instance) {
            case BLANK:
                // for simulation
                return new SimulatedHeading(kinodynamics, collection);
            default:
                RedundantGyroInterface ahrsclass = new RedundantGyroFactory(Identity.instance).get();
                return new Heading(ahrsclass);
        }
    }

    private HeadingFactory() {
        //
    }
}
