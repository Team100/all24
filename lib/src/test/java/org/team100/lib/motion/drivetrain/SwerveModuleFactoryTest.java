package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Identity;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.experiments.Experiments;

/** This just exercises the code. */
class SwerveModuleFactoryTest {
    @Test
    void testWCP() {
        Experiments experiments = new Experiments(Identity.BLANK);
        SwerveModuleFactory factory = new SwerveModuleFactory(experiments, 10);
        SwerveModule100 module = factory.WCPModule("test", 0, 0, 0, 0);
        assertNotNull(module);
        module.close();
    }

    @Test
    void testAMCAN() {
        Experiments experiments = new Experiments(Identity.BLANK);
        SwerveModuleFactory factory = new SwerveModuleFactory(experiments, 10);
        SwerveModule100 module = factory.AMCANModule("test", 0, 0, 0, 0, AnalogTurningEncoder.Drive.DIRECT);
        assertNotNull(module);
        module.close();
    }

    @Test
    void testAM() {
        Experiments experiments = new Experiments(Identity.BLANK);
        SwerveModuleFactory factory = new SwerveModuleFactory(experiments, 10);
        SwerveModule100 module = factory.AMModule("test", 0, 0, 0, 0);
        assertNotNull(module);
        module.close();
    }
}
