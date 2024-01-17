package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ChoosableProfileTest {
    @Test
    void testDefault() {
        ChoosableProfile p = new ChoosableProfile(1, 1, ChoosableProfile.Mode.PROFILE_100);
        assertEquals(ChoosableProfile.Mode.PROFILE_100, p.getSelected());
    }

    @Test
    void testOtherDefault() {
        ChoosableProfile p = new ChoosableProfile(1, 1, ChoosableProfile.Mode.PROFILE_100);
        assertEquals(ChoosableProfile.Mode.PROFILE_100, p.getSelected());
    }
}
