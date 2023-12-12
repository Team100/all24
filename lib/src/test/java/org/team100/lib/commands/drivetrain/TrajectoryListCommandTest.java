package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.motion.drivetrain.Fixture;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.trajectory.TrajectoryMaker;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class TrajectoryListCommandTest {
    private static final double kDelta = 0.001;
    private static final double kDtS = 0.02;

    @Test
    void testSimple() {

        // instantiate a real drivetrain so that it really moves

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(1, 1),
                new Translation2d(1, -1),
                new Translation2d(-1, 1),
                new Translation2d(-1, -1));

        SwerveDriveSubsystem drive = Fixture.realSwerve(kinematics);

        HolonomicDriveController3 control = new HolonomicDriveController3();

        TrajectoryListCommand c = new TrajectoryListCommand(
                drive,
                control,
                x -> List.of(TrajectoryMaker.line(kinematics, x)));
        c.initialize();
        assertEquals(0, drive.getPose().getX(), kDelta);
        c.execute();
        assertFalse(c.isFinished());
        // the trajectory takes about 2s
        for (double t = 0; t < 2; t += kDtS) {
            SimHooks.stepTiming(kDtS);
            c.execute();
            drive.periodic(); // for updateOdometry
        }
        // at goal within 5 mm
        assertEquals(1, drive.getPose().getX(), 0.005);
        assertTrue(c.isFinished());
    }
}
