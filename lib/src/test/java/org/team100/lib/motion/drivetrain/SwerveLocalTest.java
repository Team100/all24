package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Just exercises some code. */
class SwerveLocalTest {

    @Test
    void testSimple() {
        Fixture fixture = new Fixture();
        SwerveModuleCollection modules = fixture.collection;
        SwerveLocal local = fixture.swerveLocal;

        local.setChassisSpeeds(new ChassisSpeeds(), 0.02);
        assertEquals(0, modules.getDesiredStates()[0].speedMetersPerSecond, 0.001);
        local.defense();
        local.stop();
        local.setRawModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState() });
        assertEquals(0, local.speeds().vxMetersPerSecond, 0.001);
        assertEquals(0, local.positions()[0].distanceMeters, 0.001);
    }
}
