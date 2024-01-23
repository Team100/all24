package org.team100.lib.motion.drivetrain;

import org.junit.jupiter.api.AfterEach;

/** Test superclass with fixture. */
public class Fixtured {
    protected Fixture fixture = new Fixture();

    @AfterEach
    void close() {
        fixture.close();
    }
}
