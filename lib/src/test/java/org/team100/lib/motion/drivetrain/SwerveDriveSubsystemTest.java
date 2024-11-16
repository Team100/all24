package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveDriveSubsystemTest extends Fixtured implements Timeless {
    private static final double kDelta = 0.01;

    @Test
    void testWithSetpointGenerator() {
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);

        SwerveDriveSubsystem drive = fixture.drive;

        drive.resetPose(new Pose2d());

        stepTime(0.02);
        drive.periodic();
        verify(drive, 0, 0, 0);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));
        assertEquals(0.02, fixture.collection.states().frontLeft().speedMetersPerSecond);

        stepTime(0.02);
        drive.periodic();
        assertEquals(0.0004, fixture.collection.positions().frontLeft().distanceMeters, 1e-6);

        assertEquals(0.02, fixture.collection.states().frontLeft().speedMetersPerSecond);

        // the acceleration limit is applied here
        verify(drive, 0.0003, 0.019, 1.0);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));

        stepTime(0.02);
        drive.periodic();

        verify(drive, 0.001, 0.039, 1.0);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));

        stepTime(0.02);
        drive.periodic();

        verify(drive, 0.0024, 0.06, 1.0);

        drive.close();
    }

    @Test
    void testWithoutSetpointGenerator() {
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, false);
        SwerveDriveSubsystem drive = fixture.drive;

        // System.out.println("reset");
        drive.resetPose(new Pose2d());

        // System.out.println("periodic 1");
        stepTime(0.02);
        drive.periodic();

        // System.out.println("verify 1");
        verify(drive, 0, 0, 0);

        // System.out.println("set");
        // go 1 m/s in +x
        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));

        // System.out.println("periodic 2");
        stepTime(0.02);
        drive.periodic();

        // System.out.println("read distance");
        // at 1 m/s for 0.02 s, so we go 0.02 m
        assertEquals(0.02, fixture.collection.positions().frontLeft().distanceMeters, 1e-6);

        // it took 0.02 s to go from 0 m/s to 1 m/s, so we accelerated 50 m/s/s.
        // System.out.println("verify 2");
        verify(drive, 0.02, 1.00, 50.0);

        // System.out.println("periodic 3");
        stepTime(0.02);
        drive.periodic();

        // System.out.println("read 2");
        // we went a little further, no longer accelerating.
        verify(drive, 0.04, 1.00, 0.0);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));

        // System.out.println("periodic 4");
        stepTime(0.02);
        drive.periodic();

        // System.out.println("read 3");
        // a little further, but no longer accelerating
        // System.out.println("verify 3");
        verify(drive, 0.06, 1.00, 0.0);

        drive.close();
    }

    private void verify(SwerveDriveSubsystem drive, double x, double v, double a) {
        assertEquals(x, drive.getPose().getX(), kDelta);
        assertEquals(v, drive.getVelocity().x(), kDelta);
        // assertEquals(a, drive.getState().acceleration().x(), kDelta);
        assertEquals(x, drive.getState().x().x(), kDelta);
        assertEquals(v, drive.getState().x().v(), kDelta);
        // assertEquals(a, drive.getState().x().a(), kDelta);
    }
}
