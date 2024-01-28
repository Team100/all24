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
                try {
                    // the kauailabs library calls System.exit() in case
                    // it can't find this library, so check here first.
                    // System.loadLibrary("vmxHaljni");
                    RedundantGyroInterface ahrsclass = new RedundantGyroFactory(Identity.instance).get();
                    return new Heading(ahrsclass);
                } catch (UnsatisfiedLinkError e) {
                    // fall back to simulated heading for testing.
                    throw e;
                    // return new SimulatedHeading(kinodynamics, collection);
                }
        }
    }

    private HeadingFactory() {
        //
    }
}
