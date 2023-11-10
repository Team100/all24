package org.team100.lib.config;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class IdentityTest {

    @Test
    void testDefaultIdentity() {
        Identity.set("");
        assertEquals(Identity.BLANK, Identity.get());
    }

    @Test
    void testForceIdentity() {
        Identity.set("03126d76");
        assertEquals(Identity.CAMERA_DOLLY, Identity.get());
    }

    @Test
    void testUnknownIdentity() {
        Identity.set("asdf");
        assertEquals(Identity.UNKNOWN, Identity.get());
    }
}
