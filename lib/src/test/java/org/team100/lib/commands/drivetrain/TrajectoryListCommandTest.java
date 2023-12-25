package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.motion.drivetrain.Fixture;
import org.team100.lib.trajectory.TrajectoryMaker;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class TrajectoryListCommandTest {
    private static final double kDelta = 0.001;
    private static final double kDtS = 0.02;

    @Test
    void testSimple() {
        Fixture fixture = new Fixture();
        HolonomicDriveController3 control = new HolonomicDriveController3();
        TrajectoryListCommand c = new TrajectoryListCommand(
                fixture.drive,
                control,
                x -> List.of(TrajectoryMaker.line(fixture.swerveKinodynamics.getKinematics(), x)));
        c.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), kDelta);
        c.execute();
        assertFalse(c.isFinished());
        // the trajectory takes about 2s
        for (double t = 0; t < 2; t += kDtS) {
            SimHooks.stepTiming(kDtS);
            c.execute();
            fixture.drive.periodic(); // for updateOdometry
        }
        // at goal; wide tolerance due to test timing
        assertEquals(1, fixture.drive.getPose().getX(), 0.1);
        assertTrue(c.isFinished());
    }

    /**
     * See also DriveInALittleSquareTest.
     * This exists to produce useful output to graph.
     */
    @Test
    void testLowLevel() {
        Fixture fixture = new Fixture();
        HolonomicDriveController3 controller = new HolonomicDriveController3();
        TrajectoryListCommand command = new TrajectoryListCommand(
                fixture.drive,
                controller,
                x -> TrajectoryMaker.square(fixture.swerveKinodynamics.getKinematics(), x));
        fixture.experiments.testOverride(Experiment.UseSetpointGenerator, false);
        fixture.drive.periodic();
        command.initialize();
        do {
            SimHooks.stepTiming(0.02);
            fixture.drive.periodic();
            command.execute();
            double measurement = fixture.drive.moduleStates()[0].angle.getRadians();
            SwerveModuleState goal = fixture.swerveLocal.getDesiredStates()[0];
            State setpoint = fixture.swerveLocal.getSetpoints()[0];
            // this output is useful to see what's happening.
            System.out.printf("time %5.3f goal %5.3f setpoint x %5.3f setpoint v %5.3f measurement %5.3f\n",
                    command.m_timer.get(),
                    goal.angle.getRadians(),
                    setpoint.position,
                    setpoint.velocity,
                    measurement);
        } while (!command.isFinished());

    }
}
