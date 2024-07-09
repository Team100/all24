package org.team100.lib.motion.drivetrain;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
/** Just exercises some code. */
class SwerveLocalTest extends Fixtured {

    @Test
    void testSimple() {
        SwerveModuleCollection modules = fixture.collection;
        SwerveLocal local = fixture.swerveLocal;

        local.setChassisSpeeds(new ChassisSpeeds(), 0, 0.02);
        assertEquals(0, modules.getDesiredStates()[0].speedMetersPerSecond, 0.001);
        local.defense();
        local.stop();
        local.setRawModuleStates(new SwerveModuleState100[] {
                new SwerveModuleState100(),
                new SwerveModuleState100(),
                new SwerveModuleState100(),
                new SwerveModuleState100() });
        assertEquals(0, local.positions()[0].distanceMeters, 0.001);
    }
}
