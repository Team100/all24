package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class TriangleWaveTest {
    private static final double kDelta = 0.001;

    @Test
    void testUnitAmplitudeAndPeriod() {
        double amplitude = 1;
        double period = 1;
        TriangleWave w = new TriangleWave(amplitude, period);
        assertEquals(0.0, w.applyAsDouble(0.000), kDelta);
        assertEquals(0.5, w.applyAsDouble(0.125), kDelta);
        assertEquals(1.0, w.applyAsDouble(0.250), kDelta);
        assertEquals(0.5, w.applyAsDouble(0.375), kDelta);
        assertEquals(0.0, w.applyAsDouble(0.500), kDelta);
        assertEquals(-0.5, w.applyAsDouble(0.625), kDelta);
        assertEquals(-1.0, w.applyAsDouble(0.750), kDelta);
        assertEquals(-0.5, w.applyAsDouble(0.875), kDelta);
        assertEquals(0.0, w.applyAsDouble(1.000), kDelta);
    }

    @Test
    void testAmplitude() {
        double amplitude = 2;
        double period = 1;
        TriangleWave w = new TriangleWave(amplitude, period);
        assertEquals(0.0, w.applyAsDouble(0.000), kDelta);
        assertEquals(1.0, w.applyAsDouble(0.125), kDelta);
        assertEquals(2.0, w.applyAsDouble(0.250), kDelta);
        assertEquals(1.0, w.applyAsDouble(0.375), kDelta);
        assertEquals(0.0, w.applyAsDouble(0.500), kDelta);
        assertEquals(-1.0, w.applyAsDouble(0.625), kDelta);
        assertEquals(-2.0, w.applyAsDouble(0.750), kDelta);
        assertEquals(-1.0, w.applyAsDouble(0.875), kDelta);
        assertEquals(0.0, w.applyAsDouble(1.000), kDelta);
    }

    @Test
    void testPeriod() {
        double amplitude = 1;
        double period = 2;
        TriangleWave w = new TriangleWave(amplitude, period);
        assertEquals(0.0, w.applyAsDouble(0.000), kDelta);
        assertEquals(0.5, w.applyAsDouble(0.250), kDelta);
        assertEquals(1.0, w.applyAsDouble(0.500), kDelta);
        assertEquals(0.5, w.applyAsDouble(0.750), kDelta);
        assertEquals(0.0, w.applyAsDouble(1.000), kDelta);
        assertEquals(-0.5, w.applyAsDouble(1.250), kDelta);
        assertEquals(-1.0, w.applyAsDouble(1.500), kDelta);
        assertEquals(-0.5, w.applyAsDouble(1.750), kDelta);
        assertEquals(0.0, w.applyAsDouble(2.000), kDelta);
    }
}
