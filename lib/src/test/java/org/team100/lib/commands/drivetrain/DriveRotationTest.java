package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.Fixture;

class DriveRotationTest {
    private static final double kDelta = 0.001;
    private final Fixture fixture = new Fixture();
    private final double desiredRotation = 1;

    @AfterEach
    void close() {
        fixture.close();
    }

    @Test
    void testSimple() {
        Supplier<Double> rot = () -> desiredRotation;
        DriveRotation command = new DriveRotation(fixture.drive, rot);
        command.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), kDelta);
        command.execute();
        command.end(false);
    }
}
