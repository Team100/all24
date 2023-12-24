package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Just exercises some code. */
class SwerveLocalTest {

    @Test
    void testSimple() {
        Experiments experiments = new Experiments(Identity.BLANK);
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        MockSwerveModuleCollection modules = new MockSwerveModuleCollection();
        SwerveLocal local = new SwerveLocal(experiments, swerveKinodynamics, modules);
        local.setChassisSpeeds(new ChassisSpeeds());
        assertEquals(0, modules.m_targetModuleStates[0].speedMetersPerSecond, 0.001);
        local.defense();
        local.stop();
        assertTrue(modules.stopped);
        local.setRawModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState() });
        assertEquals(0, local.speeds().vxMetersPerSecond, 0.001);
        assertEquals(0, local.positions()[0].distanceMeters, 0.001);
    }
}
