package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

 class ChoosableProfileTest {
    @Test
    void testDefault() {
        ChoosableProfile p = new ChoosableProfile(1, 1, 0, ChoosableProfile.Mode.TRAPEZOID);
        assertEquals(ChoosableProfile.Mode.TRAPEZOID, p.getSelected());
    }

    @Test
    void testOtherDefault() {
        ChoosableProfile p = new ChoosableProfile(1, 1, 0, ChoosableProfile.Mode.MOTION_PROFILE);
        assertEquals(ChoosableProfile.Mode.MOTION_PROFILE, p.getSelected());
    }
}
