package org.team100.lib.motion.drivetrain.module;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.hal.HAL;

/** This just exercises the code. */
class SwerveModuleFactoryTest {
    @Test
    void testWCP() {
        HAL.initialize(500, 0);
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forTest();
        SwerveModule100 module = WCPSwerveModule100.get("test", 0, 0, 0, 0, 0, k);
        assertNotNull(module);
        module.close();
        HAL.shutdown();
    }

    @Test
    void testAMCAN() {
        HAL.initialize(500, 0);
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forTest();
        SwerveModule100 module = AMCANSwerveModule100.get("test", 0, 0, 0, 0, 0, AnalogTurningEncoder.Drive.DIRECT, k);
        assertNotNull(module);
        module.close();
        HAL.shutdown();
    }

    @Test
    void testAM() {
        HAL.initialize(500, 0);
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forTest();
        SwerveModule100 module = AMSwerveModule100.get("test", 0, 0, 0, 0, 0, k);
        assertNotNull(module);
        module.close();
        HAL.shutdown();
    }
}
