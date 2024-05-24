package org.team100.lib.async;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class AsyncTest {
    private int counter = 0;

    @Test
    void testSimple() throws InterruptedException {
        AsyncFactory.get().addPeriodic(() -> counter += 1, 0.1);
        Thread.sleep(1000); // 1 sec
        assertEquals(9, counter, 1);
    }
}
