package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.Fixture;
import org.team100.lib.motion.drivetrain.MockSwerveDriveSubsystem;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class DriveInALittleSquareTest {
    private static final double kDelta = 0.001;

    /** Confirm that the steering commands are simple steps. */
    @Test
    void testSteering() {
        MockSwerveDriveSubsystem swerve = new MockSwerveDriveSubsystem();
        DriveInALittleSquare command = new DriveInALittleSquare(swerve);
        command.initialize();
        command.execute();
        assertEquals(DriveInALittleSquare.State.DRIVING, command.m_state);
        assertEquals(0, command.speedM_S.velocity, kDelta);
        assertEquals(0, command.m_goal.getRadians(), kDelta);
        assertEquals(2, command.m_driveProfile.totalTime(), kDelta);
        assertEquals(0, swerve.states[0].speedMetersPerSecond, kDelta);
        assertEquals(0, swerve.states[0].angle.getRadians(), kDelta);
        SimHooks.stepTiming(3.2);
        command.execute();
        assertEquals(DriveInALittleSquare.State.STEERING, command.m_state);
        assertEquals(0, command.speedM_S.velocity, kDelta);
        assertEquals(Math.PI / 2, command.m_goal.getRadians(), kDelta);
        // the mock drivetrain just records what it's told to do
        assertEquals(0, swerve.states[0].speedMetersPerSecond, kDelta);
        assertEquals(Math.PI / 2, swerve.states[0].angle.getRadians(), kDelta);
    }

    @Test
    void testLowLevel() {
        Fixture fixture = new Fixture();
        DriveInALittleSquare command = new DriveInALittleSquare(fixture.drive);
        command.initialize();
        assertEquals(DriveInALittleSquare.State.DRIVING, command.m_state);
        command.execute();
        assertEquals(DriveInALittleSquare.State.DRIVING, command.m_state);
        // this seems subject to jitter
        assertEquals(0, command.speedM_S.velocity, 0.1);
        assertEquals(0, command.m_goal.getRadians(), kDelta);
        assertEquals(2, command.m_driveProfile.totalTime(), kDelta);
        assertEquals(0, fixture.drive.moduleStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(0, fixture.drive.moduleStates()[0].angle.getRadians(), kDelta);
        SimHooks.stepTiming(3.2);
        command.execute();
        assertEquals(DriveInALittleSquare.State.STEERING, command.m_state);
        assertEquals(0, command.speedM_S.velocity, kDelta);
        assertEquals(Math.PI / 2, command.m_goal.getRadians(), kDelta);
        assertEquals(0, fixture.drive.moduleStates()[0].speedMetersPerSecond, kDelta);
        assertFalse(fixture.drive.atGoal()[0]);
        // the modules have not moved yet.
        assertEquals(0, fixture.drive.moduleStates()[0].angle.getRadians(), 0.01);

        // SimHooks.stepTiming(0.02);
        // command.execute();
        // assertEquals(0, fixture.drive.moduleStates()[0].angle.getRadians(), kDelta);

        // wait a half second.
        // this duration matches the profile and the feedback settings
        // which involve a bit of lag and overshoot.
        for (int i = 0; i < 25; i++) {
            SimHooks.stepTiming(0.02);
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
        }
        // after that time, the wheels have rotated.
        // note the controller tolerance is 
        assertEquals(Math.PI / 2, fixture.drive.moduleStates()[0].angle.getRadians(), 0.01);
        assertTrue(fixture.drive.atGoal()[0]);
        // and we're driving again
        assertEquals(DriveInALittleSquare.State.DRIVING, command.m_state);
        // we're not quite motionless, we're already going a little.
        // there's no specific test here because the velocity seems to depend
        // on the timing in the simulation
        assertTrue(command.speedM_S.velocity > 0);
    }

}
