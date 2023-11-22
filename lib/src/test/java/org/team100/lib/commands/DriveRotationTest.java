package org.team100.lib.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;

class DriveRotationTest {
    double desiredRotation = 1;

    @Test
    void testSimple() {
        MockSwerveDriveSubsystem robotDrive = new MockSwerveDriveSubsystem();

        Supplier<Double> rot = () -> desiredRotation;

        DriveRotation command = new DriveRotation(
                robotDrive, rot);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        assertEquals(1, robotDrive.twist.dtheta, 0.001);
        command.end(false);
    }
}
