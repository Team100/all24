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
    void testAccel() {
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);

        SwerveDriveSubsystem drive = fixture.drive;

        drive.resetPose(new Pose2d());

        stepTime(0.02);
        drive.periodic();

        // no command, initial state.
        verify(drive, 0, 0, 0);

        // set the motor speed...
        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));
        // this state is hidden...
        assertEquals(0.02, fixture.collection.states()[0].speedMetersPerSecond);

        // smaller step 
        stepTime(0.01);
        drive.periodic();
        stepTime(0.01);
        drive.periodic();
        assertEquals(0.0004, fixture.collection.positions()[0].distanceMeters, 1e-6);

        // until after periodic...
        assertEquals(0.02, fixture.collection.states()[0].speedMetersPerSecond);

        // new state should be accelerating
        verify(drive, 0.0003, 0.019, 0.006);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));

        stepTime(0.02);
        drive.periodic();

        // keep accelerating

        verify(drive, 0.001, 0.039, 0.019);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));

        stepTime(0.02);
        drive.periodic();

        // keep accelerating
        verify(drive, 0.0024, 0.06, 0.039);

        drive.close();
    }

    private void verify(SwerveDriveSubsystem drive, double x, double v, double a) {
        assertEquals(x, drive.getState().pose().getX(), kDelta);
        assertEquals(v, drive.getState().velocity().x(), kDelta);
        assertEquals(a, drive.getState().acceleration().x(), kDelta);
        assertEquals(x, drive.getState().x().x(), kDelta);
        assertEquals(v, drive.getState().x().v(), kDelta);
        assertEquals(a, drive.getState().x().a(), kDelta);
    }
}
