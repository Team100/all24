package org.team100.frc2024.motion.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.frc2024.Timeless2024;

class DrumShooterTest implements Timeless2024 {
    private static final double kDelta = 0.001;

    @Test
    void testAngle() {
        DrumShooter s = new DrumShooter(0, 0, 0, 0);
        for (int i = 0; i < 50; ++i) {
            s.setAngle(1.0);
            stepTime(0.02);
        }
        assertEquals(1, s.getAngleRad(), 0.001);
    }

    @Test
    void testRollers() {
        DrumShooter s = new DrumShooter(0, 0, 0, 0);
        // 0.5 sec to spin up
        for (int i = 0; i <= 25; ++i) {
            s.forward();
            stepTime(0.02);

        }
        // 17.5 is the average of the two drums in m/s
        assertEquals(17.5, s.getVelocity(), kDelta);
    }

}
