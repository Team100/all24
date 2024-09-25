package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.logging.TestLogger;
import org.team100.lib.logging.SupplierLogger2;

class DriveRotationTest extends Fixtured {
    private static final double kDelta = 0.001;
    private static final SupplierLogger2 logger = new TestLogger().getSupplierLogger();
    private final double desiredRotation = 1;

    @Test
    void testSimple() {
        Supplier<Double> rot = () -> desiredRotation;
        DriveRotation command = new DriveRotation(logger, fixture.drive, rot);
        command.initialize();
        assertEquals(0, fixture.drive.getState().pose().getX(), kDelta);
        command.execute();
        command.end(false);
    }
}
