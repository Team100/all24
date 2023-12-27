package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.Fixture;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.sensors.MockHeading;

import edu.wpi.first.wpilibj.simulation.SimHooks;

class RotateTest {
    private static final double kDelta = 0.02;

    Fixture fixture = new Fixture();

    @Test
    void testRotate() {
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);
        SwerveDriveSubsystem swerveDriveSubsystem = fixture.drive;
        MockHeading heading = new MockHeading();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // remember the test rotation rate is *very* slow.
        assertEquals(2.828, swerveKinodynamics.getMaxAngleSpeedRad_S(), 0.001);
        double targetAngle = Math.PI / 2;

        Rotate rotate = new Rotate(
                swerveDriveSubsystem,
                heading,
                swerveKinodynamics,
                targetAngle);

        rotate.initialize();

        assertEquals(0, rotate.refTheta.position, kDelta); // at start
        assertEquals(0, fixture.drive.desiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(0, fixture.drive.desiredStates()[0].angle.getRadians(), kDelta);

        // steering
        for (int i = 0; i < 18; ++i) {
            SimHooks.stepTimingAsync(0.02);
            fixture.drive.periodic();
            rotate.execute();
        }
        // there's no translation
        assertEquals(0, rotate.m_controller.error().getX(), kDelta);
        assertEquals(0, rotate.m_controller.error().getY(), kDelta);
        // the profile is held until steering is done
        assertEquals(0, rotate.m_controller.error().getRotation().getRadians(), 0.01);
        assertEquals(0, rotate.refTheta.position, 0.01);
        // now we're ready to start rotating
        assertEquals(0, fixture.drive.desiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(-Math.PI/4, fixture.drive.desiredStates()[0].angle.getRadians(), 0.1);

        // finished steering, start rotating
        for (int i = 0; i < 25; ++i) {
            SimHooks.stepTimingAsync(0.02);
            fixture.drive.periodic();
            rotate.execute();
        }
        assertEquals(1, rotate.refTheta.position, 0.2);
        assertEquals(-0.512, fixture.drive.desiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(-Math.PI/4, fixture.drive.desiredStates()[0].angle.getRadians(), 0.1);

        // should be done rotating now
        for (int i = 0; i < 25; ++i) {
            SimHooks.stepTimingAsync(0.02);
            fixture.drive.periodic();
            rotate.execute();
        }

        assertEquals(Math.PI/2, rotate.refTheta.position, kDelta);
        // assertEquals(-0.403, fixture.drive.desiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(-Math.PI/4, fixture.drive.desiredStates()[0].angle.getRadians(), kDelta);

        for (int i = 0; i < 100; ++i) {
            SimHooks.stepTimingAsync(0.02);
            fixture.drive.periodic();
            rotate.execute();
        }

        assertTrue(rotate.isFinished());

        rotate.end(false);
        assertEquals(0, fixture.drive.desiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(-Math.PI/4, fixture.drive.desiredStates()[0].angle.getRadians(), kDelta);

    }
}
