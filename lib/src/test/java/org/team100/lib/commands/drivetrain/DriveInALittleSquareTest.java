package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.Fixture;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.Util;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class DriveInALittleSquareTest {
    boolean dump = true;
    private static final double kDelta = 0.001;

    Fixture fixture = new Fixture();

    /** Confirm that the steering commands are simple steps. */
    @Test
    void testSteering() {
        SwerveDriveSubsystem swerve = fixture.drive;
        DriveInALittleSquare command = new DriveInALittleSquare(swerve);
        command.initialize();

        // the first time we call execute, drive doesn't yet know it's at the goal
        fixture.drive.periodic();
        command.execute();
        // initially steering
        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);
        assertEquals(0, command.speedM_S.v(), kDelta);
        assertEquals(0, command.m_goal.getRadians(), kDelta);
        assertEquals(0, swerve.desiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(0, swerve.desiredStates()[0].angle.getRadians(), kDelta);

        // the second time, it knows, so we switch to driving.
        fixture.drive.periodic();
        command.execute();
        assertEquals(DriveInALittleSquare.DriveState.DRIVING, command.m_state);
        assertEquals(0.008, command.speedM_S.v(), 0.01);
        assertEquals(0, command.m_goal.getRadians(), kDelta);
        assertEquals(0, swerve.desiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(0, swerve.desiredStates()[0].angle.getRadians(), kDelta);

        // step through the driving phase
        SimHooks.stepTimingAsync(2.5);
        fixture.drive.periodic();
        command.execute();
        // now we should be steering again
        SimHooks.stepTimingAsync(0.02);
        fixture.drive.periodic();
        command.execute();
        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);
        assertEquals(0, command.speedM_S.v(), kDelta);
        assertEquals(Math.PI/2, command.m_goal.getRadians(), kDelta);
        assertEquals(0, swerve.desiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(Math.PI/2, swerve.desiredStates()[0].angle.getRadians(), kDelta);
    }

    @Test
    void testLowLevel() {
        DriveInALittleSquare command = new DriveInALittleSquare(fixture.drive);
        command.initialize();
        // first align the wheels in case they're not already aligned.
        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);
        fixture.drive.periodic();
        command.execute();
        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);
        assertEquals(0, command.m_goal.getRadians(), kDelta);

        // a little while later we should be driving
        SimHooks.stepTimingAsync(0.1);
        fixture.drive.periodic();
        command.execute();
        assertEquals(DriveInALittleSquare.DriveState.DRIVING, command.m_state);
        assertEquals(0.1, command.speedM_S.v(), 0.05);
        assertEquals(0, command.m_goal.getRadians(), kDelta);
        // the actual speed lags slightly
        assertEquals(0.005, fixture.drive.moduleStates()[0].speedMetersPerSecond, 0.005);

        // drive to the next corner
        for (double t = 0; t < 2.1; t += 0.02) {
            SimHooks.stepTimingAsync(0.02);
            fixture.drive.periodic();
            command.execute();
            double measurement = fixture.drive.moduleStates()[0].speedMetersPerSecond;
            if (dump)
                Util.printf("t %5.3f measurement %5.3f\n", t, measurement);
        }

        // steer
        SimHooks.stepTimingAsync(0.02);
        fixture.drive.periodic();
        command.execute();
        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);
        assertEquals(0, command.speedM_S.v(), kDelta);
        assertEquals(Math.PI / 2, command.m_goal.getRadians(), kDelta);
        assertEquals(0, fixture.drive.moduleStates()[0].speedMetersPerSecond, kDelta);
        assertFalse(fixture.drive.atGoal()[0]);

        // the modules are not there yet
        // assertEquals(0.06, fixture.drive.moduleStates()[0].angle.getRadians(), 0.06);

        // wait a half second.
        for (double t = 0; t < 0.5; t += 0.02) {
            SimHooks.stepTimingAsync(0.02);
            fixture.drive.periodic();
            command.execute();
            double measurement = fixture.drive.moduleStates()[0].angle.getRadians();
            SwerveModuleState goal = fixture.swerveLocal.getDesiredStates()[0];
            State100 setpoint = fixture.swerveLocal.getSetpoints()[0];
            // this output is useful to see what's happening.
            if (dump)
                Util.printf("t %5.3f goal %5.3f setpoint x %5.3f setpoint v %5.3f measurement %5.3f\n",
                        t,
                        goal.angle.getRadians(),
                        setpoint.x(),
                        setpoint.v(),
                        measurement);
        }
        // after that time, the wheels have rotated.
        // note the controller tolerance is
        assertEquals(Math.PI / 2, fixture.drive.moduleStates()[0].angle.getRadians(), 0.01);
        assertTrue(fixture.drive.atGoal()[0]);
        // and we're driving again
        assertEquals(DriveInALittleSquare.DriveState.DRIVING, command.m_state);
        // we're not quite motionless, we're already going a little.
        // there's no specific test here because the velocity seems to depend
        // on the timing in the simulation
        assertTrue(command.speedM_S.v() > 0);
    }

}
