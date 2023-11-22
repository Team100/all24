package org.team100.lib.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SpeedLimits;

import edu.wpi.first.math.geometry.Twist2d;

class DriveManuallyTest {
    ManualMode.Mode desiredMode = null;
    Twist2d desiredTwist = new Twist2d(1,0,0);

    @Test
    void testSimple() {
        Supplier<ManualMode.Mode> mode = () -> desiredMode;
        Supplier<Twist2d> twistSupplier = () -> desiredTwist;
        MockSwerveDriveSubsystem robotDrive = new MockSwerveDriveSubsystem();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);

        DriveManually command = new DriveManually(
                mode,
                twistSupplier,
                robotDrive,
                speedLimits);

        command.initialize();

        desiredMode = ManualMode.Mode.MODULE_STATE;
        command.execute();
        assertEquals(1, robotDrive.speeds().vxMetersPerSecond, 0.001);

        desiredMode = ManualMode.Mode.ROBOT_RELATIVE_CHASSIS_SPEED;
        command.execute();
        assertEquals(1, robotDrive.speeds().vxMetersPerSecond, 0.001);

        desiredMode = ManualMode.Mode.FIELD_RELATIVE_TWIST;
        command.execute();
        assertEquals(1, robotDrive.twist.dx, 0.001);

        command.end(false);
    }

}
