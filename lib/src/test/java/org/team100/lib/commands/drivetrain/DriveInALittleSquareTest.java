package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;

import org.junit.jupiter.api.Test;
import org.team100.lib.commands.drivetrain.DriveInALittleSquare.DriveState;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.logging.TestLogger;
import org.team100.lib.logging.SupplierLogger;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Util;

class DriveInALittleSquareTest extends Fixtured implements Timeless {
    boolean dump = false;
    private static final double kDelta = 0.001;
    private static final SupplierLogger logger = new TestLogger().getSupplierLogger();

    /** Confirm that the steering commands are simple steps. */
    @Test
    void testSteering() {
        SwerveDriveSubsystem swerve = fixture.drive;
        DriveInALittleSquare command = new DriveInALittleSquare(logger, swerve);
        command.initialize();
        // the first time we call execute, drive doesn't yet know it's at the goal
        stepTime(0.02);
        fixture.drive.periodic();
        command.execute100(0.02);
        // initially steering
        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);
        assertEquals(0, command.m_setpoint.v(), kDelta);
        assertEquals(0, command.m_goal.getRadians(), kDelta);
        assertEquals(0, swerve.getSwerveLocal().getDesiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(0, swerve.getSwerveLocal().getDesiredStates()[0].angle.get().getRadians(), kDelta);

        // the second time, it knows, so we switch to driving.
        stepTime(0.02);
        fixture.drive.periodic();
        command.execute100(0.02);
        assertEquals(DriveInALittleSquare.DriveState.DRIVING, command.m_state);
        assertEquals(0.02, command.m_setpoint.v(), kDelta);
        assertEquals(0, command.m_goal.getRadians(), kDelta);
        assertEquals(0.02, swerve.getSwerveLocal().getDesiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(0, swerve.getSwerveLocal().getDesiredStates()[0].angle.get().getRadians(), kDelta);

        // step through the driving phase
        stepTime(2.5);
        fixture.drive.periodic();
        command.execute100(2.5);
        // now we should be steering again
        stepTime(0.02);
        fixture.drive.periodic();
        command.execute100(0.02);
        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);
        assertEquals(0, command.m_setpoint.v(), kDelta);
        assertEquals(Math.PI / 2, command.m_goal.getRadians(), kDelta);
        assertEquals(0, swerve.getSwerveLocal().getDesiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(Math.PI / 2, swerve.getSwerveLocal().getDesiredStates()[0].angle.get().getRadians(), kDelta);
    }

    @Test
    void testLowLevel() {
        // there's some weird thing with getFPGATimestamp in tests mode
        // that messes up how the simulated encoder measures position,
        // and somehow magically the line below fixes it.
        assertEquals(0.0, fixture.drive.getSwerveLocal().positions()[0].distanceMeters, 0.005);

        DriveInALittleSquare command = new DriveInALittleSquare(logger, fixture.drive);
        command.initialize();

        // first align the wheels in case they're not already aligned.
        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);

        fixture.drive.periodic();
        command.execute100(0);
        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);
        assertEquals(0, command.m_goal.getRadians(), kDelta);

        // a little while later we should be driving
        // at this point the speed is still zero
        assertEquals(0.0, fixture.drive.getSwerveLocal().states()[0].speedMetersPerSecond, 0.005);
        stepTime(0.1);
        fixture.drive.periodic();
        // this changes the speed
        command.execute100(0.1);
        // big jump in speed here since dt is so big
        // max accel is 1 so waiting 0.1s means 0.1m/s
        assertEquals(DriveInALittleSquare.DriveState.DRIVING, command.m_state);
        assertEquals(0.1, command.m_setpoint.v(), 0.05);
        assertEquals(0, command.m_goal.getRadians(), kDelta);
        assertEquals(0.1, fixture.drive.getSwerveLocal().states()[0].speedMetersPerSecond, 0.005);
        // position isn't updated until the next time-step.
        // assertEquals(0.0,
        // fixture.drive.getSwerveLocal().positions()[0].distanceMeters, 0.005);

        // drive to the next corner. at 1 m/s/s this should be a triangular
        // profile that takes exactly 2 sec total but we started at 0.1 so 1.9
        for (double t = 0; t < 1.9; t += 0.02) {
            stepTime(0.02);
            fixture.drive.periodic();
            command.execute100(0.02);
            double speed = fixture.drive.getSwerveLocal().states()[0].speedMetersPerSecond;
            double distance = fixture.drive.getSwerveLocal().positions()[0].distanceMeters;
            DriveState state = command.m_state;
            if (dump)
                Util.printf("t %5.3f state %s speed %5.3f distance %5.3f\n", t, state.toString(), speed, distance);
        }

        // steer
        stepTime(0.02);
        fixture.drive.periodic();
        command.execute100(0.02);
        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);
        assertEquals(0, command.m_setpoint.v(), kDelta);
        assertEquals(Math.PI / 2, command.m_goal.getRadians(), kDelta);
        assertEquals(0, fixture.drive.getSwerveLocal().states()[0].speedMetersPerSecond, kDelta);
        assertFalse(fixture.drive.getSwerveLocal().atGoal()[0]);

        // wait a half second.
        for (double t = 0; t < 0.5; t += 0.02) {
            stepTime(0.02);
            fixture.drive.periodic();
            command.execute100(0.02);
            double measurement = fixture.drive.getSwerveLocal().states()[0].angle.get().getRadians();
            SwerveModuleState100 goal = fixture.swerveLocal.getDesiredStates()[0];
            State100 setpoint = fixture.swerveLocal.getSetpoints()[0];
            // this output is useful to see what's happening.
            if (dump)
                Util.printf("t %5.3f goal %5.3f setpoint x %5.3f setpoint v %5.3f measurement %5.3f\n",
                        t,
                        goal.angle.get().getRadians(),
                        setpoint.x(),
                        setpoint.v(),
                        measurement);
        }
        // after that time, the wheels have rotated.
        // note the controller tolerance is
        assertEquals(Math.PI / 2, fixture.drive.getSwerveLocal().states()[0].angle.get().getRadians(), 0.01);
        assertTrue(fixture.drive.getSwerveLocal().atGoal()[0]);
        // and we're driving again
        assertEquals(DriveInALittleSquare.DriveState.DRIVING, command.m_state);
        // we're not quite motionless, we're already going a little.
        // there's no specific test here because the velocity seems to depend
        // on the timing in the simulation
        assertTrue(command.m_setpoint.v() > 0);
    }

}
