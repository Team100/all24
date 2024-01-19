package org.team100.lib.testing;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/**
 * Sets up FPGA time stepping for unit tests.
 * 
 * Pausing the timer makes the tests deterministic.
 */
public class TimelessTest {

    @BeforeEach
    void before() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
    }

    @AfterEach
    void after() {
        SimHooks.resumeTiming();
        HAL.shutdown();
    }

    protected void stepTime(double t) {
        SimHooks.stepTiming(t);
    }

}
