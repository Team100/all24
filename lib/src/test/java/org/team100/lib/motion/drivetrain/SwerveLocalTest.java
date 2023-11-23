package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiments;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Just exercises some code. */
class SwerveLocalTest {
    SwerveModuleState[] m_targetModuleStates;
    boolean stopped = false;

    @Test
    void testSimple() {
        Experiments experiments = new Experiments(Identity.BLANK);
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(0.1, 0.1),
                new Translation2d(0.1, -0.1),
                new Translation2d(-0.1, 0.1),
                new Translation2d(-0.1, -0.1));
        SwerveModuleCollectionInterface modules = new SwerveModuleCollectionInterface() {

            @Override
            public void setDesiredStates(SwerveModuleState[] targetModuleStates) {
                m_targetModuleStates = targetModuleStates;
            }

            @Override
            public SwerveModulePosition[] positions() {
                return new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                };
            }

            @Override
            public SwerveModuleState[] states() {
                return m_targetModuleStates;
            }

            @Override
            public void stop() {
                stopped = true;
            }

            @Override
            public void close() {
                //
            }
        };
        SwerveLocal local = new SwerveLocal(experiments, speedLimits, kinematics, modules);
        local.setChassisSpeeds(new ChassisSpeeds());
        assertEquals(0, m_targetModuleStates[0].speedMetersPerSecond, 0.001);
        local.defense();
        local.stop();
        assertTrue(stopped);
        local.setRawModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState() });
        assertEquals(0, local.speeds().vxMetersPerSecond, 0.001);
        assertEquals(0, local.positions()[0].distanceMeters, 0.001);
    }
}
