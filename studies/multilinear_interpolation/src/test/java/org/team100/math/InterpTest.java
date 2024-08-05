package org.team100.math;

import java.lang.reflect.InvocationTargetException;

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

    @Test
    void testDemo() throws InvocationTargetException, InterruptedException {
        Interp interp = new Interp();
        interp.demo();
    }
}
