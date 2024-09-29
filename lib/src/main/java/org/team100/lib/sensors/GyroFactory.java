package org.team100.lib.sensors;

import org.team100.lib.async.AsyncFactory;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.util.Util;

/**
 * Produces real or simulated gyros depending on identity.
 */
public class GyroFactory {

    public static Gyro get(
            SupplierLogger2 parent,
            SwerveKinodynamics kinodynamics,
            SwerveModuleCollection collection,
            AsyncFactory asyncFactory) {
        switch (Identity.instance) {
            case COMP_BOT:
            case SWERVE_ONE:
                try {
                    // the kauailabs library calls System.exit() in case
                    // it can't find this library, so check here first.
                    // this seems only to be a problem in some test or
                    // simulation scenarios.
                    // System.loadLibrary("vmxHaljni");
                    return new SelectGyro(
                            new NTGyro(),
                            new SingleNavXGyro(parent, asyncFactory.get()),
                            () -> Experiments.instance.enabled(Experiment.NetworkGyro));
                } catch (UnsatisfiedLinkError e) {
                    // fall back to simulated heading for testing.
                    Util.warn("No NavX Library!");
                    throw e;
                    // return new SimulatedHeading(kinodynamics, collection);
                }
            default:
                // for simulation
                return new SelectGyro(
                        new NTGyro(),
                        new SimulatedGyro(kinodynamics, collection),
                        () -> Experiments.instance.enabled(Experiment.NetworkGyro));

        }
    }

    private GyroFactory() {
        //
    }
}
