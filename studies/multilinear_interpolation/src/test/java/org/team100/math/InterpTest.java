package org.team100.math;

import org.junit.jupiter.api.Test;

class InterpTest {
    @Test
    void testKriging() {
        Interp interp = new Interp();
        interp.kriging();
    }
    @Test
    void testRbf() {
        Interp interp = new Interp();
        interp.rbf();
    }
    
}
