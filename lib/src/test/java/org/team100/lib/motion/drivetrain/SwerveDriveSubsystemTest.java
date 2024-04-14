package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveDriveSubsystemTest extends Fixtured implements Timeless {
    private static final double kDelta = 0.001;

    @Test
    void testAccel() {
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);

        SwerveDriveSubsystem drive = fixture.drive;

        drive.resetPose(new Pose2d());

        stepTime(0.02);
        drive.periodic();

        // no command, initial state.
        verify(drive, 0,0,0);

        // set the motor speed...
        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0), 0.02);
        // this state is hidden...
        assertEquals(0.02, fixture.collection.states()[0].speedMetersPerSecond);

        stepTime(0.02);
        drive.periodic();

        // until after periodic...
        assertEquals(0.02, fixture.collection.states()[0].speedMetersPerSecond);

        // new state should be accelerating
        verify(drive, 0.0006, 0.02, 1);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0), 0.02);

        stepTime(0.02);
        drive.periodic();

        // keep accelerating

        verify(drive, 0.0015, 0.04, 1);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0), 0.02);

        stepTime(0.02);
        drive.periodic();

        // keep accelerating
        verify(drive, 0.002, 0.06, 1);

        drive.close();
    }

    private void verify(SwerveDriveSubsystem drive, double x, double v, double a) {
        assertEquals(x, drive.getPose().getX(), kDelta);        
        assertEquals(v, drive.getVelocity().x(), kDelta);
        assertEquals(a, drive.getAcceleration().x(), kDelta);
        assertEquals(x, drive.getState().x().x(), kDelta);
        assertEquals(v, drive.getState().x().v(), kDelta);
        assertEquals(a, drive.getState().x().a(), kDelta);
    }
}
