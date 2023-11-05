package org.team100.lib.space;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;

class PathTest {
    @Test
    void testIsBetter() {
        List<Matrix<N1, N1>> a1 = new ArrayList<>();
        a1.add(VecBuilder.fill(0));
        a1.add(VecBuilder.fill(1));
        List<Matrix<N1, N1>> b1 = new ArrayList<>();
        b1.add(VecBuilder.fill(2));
        b1.add(VecBuilder.fill(3));
        Path<N1> p1 = new Path<>(2, a1, b1);
        assertEquals(2, p1.getDistance());

        List<Matrix<N1, N1>> a2 = new ArrayList<>();
        a2.add(VecBuilder.fill(0));
        a2.add(VecBuilder.fill(2));
        List<Matrix<N1, N1>> b2 = new ArrayList<>();
        b2.add(VecBuilder.fill(3));
        b2.add(VecBuilder.fill(4));
        Path<N1> p2 = new Path<>(3, a2, b2);
        assertEquals(3, p2.getDistance());

        assertTrue(Path.isBetter(p1,p2));
    }

    @Test
    void testSingleIsBetter() {
        List<SinglePath.Link<N1>> l1 = new ArrayList<>();
        l1.add(new SinglePath.Link<>(VecBuilder.fill(0), VecBuilder.fill(1), 1));
        l1.add(new SinglePath.Link<>(VecBuilder.fill(1), VecBuilder.fill(2), 1));
        l1.add(new SinglePath.Link<>(VecBuilder.fill(2), VecBuilder.fill(3), 1));
        SinglePath<N1> p1 = new SinglePath<>(l1);
        assertEquals(3, p1.getDistance());

        List<SinglePath.Link<N1>> l2 = new ArrayList<>();
        l2.add(new SinglePath.Link<>(VecBuilder.fill(0), VecBuilder.fill(2), 2));
        l2.add(new SinglePath.Link<>(VecBuilder.fill(2), VecBuilder.fill(3), 1));
        l2.add(new SinglePath.Link<>(VecBuilder.fill(3), VecBuilder.fill(4), 1));
        SinglePath<N1> p2 = new SinglePath<>(l2);
        assertEquals(4, p2.getDistance());

        assertTrue(p1.getDistance() < p2.getDistance());
    }

}
