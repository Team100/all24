package org.team100.lib.motion.drivetrain.module;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.encoder.drive.Drive;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.testing.Timeless;

/** This just exercises the code. */
class SwerveModuleFactoryTest implements Timeless {
    @Test
    void testWCP() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forTest();
        SwerveModule100 module = WCPSwerveModule100.get("test", 0, 0, WCPSwerveModule100.DriveRatio.MEDIUM,
                AnalogTurningEncoder.class, 0, 0, 0, k, Drive.DIRECT, MotorPhase.FORWARD);
        assertNotNull(module);
        module.close();
    }

    @Test
    void testAMCAN() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forTest();
        SwerveModule100 module = AMCANSwerveModule100.get("test", 0, 0, 0, 0, 0, Drive.DIRECT, k);
        assertNotNull(module);
        module.close();
    }

    @Test
    void testAM() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forTest();
        FeedforwardConstants FeedforwardConstants = new FeedforwardConstants();
        PIDConstants pidConstants = new PIDConstants(1);
        SwerveModule100 module = AMSwerveModule100.get("test", 0, 0, 0, 0, 0, k, pidConstants, FeedforwardConstants);
        assertNotNull(module);
        module.close();
    }
}
