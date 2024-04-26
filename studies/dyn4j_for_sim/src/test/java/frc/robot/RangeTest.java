package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.sim.Range;

class RangeTest {
    @Test
    void testDisjoint() {
        Range r1 = new Range(0, 1);
        Range r2 = new Range(2, 3);
        assertFalse(r1.overlaps(r2));
        assertFalse(r2.overlaps(r1));
    }

    @Test
    void testOverlap() {
        Range r1 = new Range(0, 2);
        Range r2 = new Range(1, 3);
        assertTrue(r1.overlaps(r2));
        assertTrue(r2.overlaps(r1));
    }

    @Test
    void testContains() {
        Range r1 = new Range(0, 3);
        Range r2 = new Range(1, 2);
        assertTrue(r1.contains(r2));
        assertFalse(r2.contains(r1));

    }
}
