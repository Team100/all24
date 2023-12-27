package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;

import edu.wpi.first.hal.HAL;

/** This just exercises the code. */
class SwerveModuleFactoryTest {
    @Test
    void testWCP() {
        HAL.initialize(500, 0);
        SwerveModuleFactory factory = new SwerveModuleFactory(10);
        SwerveModule100 module = factory.WCPModule("test", 0, 0, 0, 0);
        assertNotNull(module);
        module.close();
        HAL.shutdown();
    }

    @Test
    void testAMCAN() {
        HAL.initialize(500, 0);
        SwerveModuleFactory factory = new SwerveModuleFactory(10);
        SwerveModule100 module = factory.AMCANModule("test", 0, 0, 0, 0, AnalogTurningEncoder.Drive.DIRECT);
        assertNotNull(module);
        module.close();
        HAL.shutdown();
    }

    @Test
    void testAM() {
        HAL.initialize(500, 0);
        SwerveModuleFactory factory = new SwerveModuleFactory(10);
        SwerveModule100 module = factory.AMModule("test", 0, 0, 0, 0);
        assertNotNull(module);
        module.close();
        HAL.shutdown();
    }
}
