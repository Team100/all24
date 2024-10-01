package org.team100.lib.motion.drivetrain;

import org.junit.jupiter.api.AfterEach;

public class RealisticFixtured {
    protected RealisticFixture fixture = new RealisticFixture();

    @AfterEach
    void close() {
        fixture.close();
    }
}
