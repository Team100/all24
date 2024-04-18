package org.team100.frc2024;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/** A copy of Timeless from lib test, because frc2024 can't see it */
public interface Timeless2024 {

    @BeforeEach
    default void pauseTiming() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
    }

    @AfterEach
    default void resumeTiming() {
        SimHooks.resumeTiming();
        HAL.shutdown();
    }

    default void stepTime(double t) {
        SimHooks.stepTiming(t);
    }

}
