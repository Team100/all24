package org.team100.lib.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.LinearSystem;

/** Make sure I know what the plant-inversion feedforward thing is doing. */
class FeedforwardTest {
    /** This is the constant-acceleration case. */
    @Test
    void testFeedforward() {
        double dtSeconds = 0.02;

        final Matrix<N2, N2> A = Matrix.mat(Nat.N2(), Nat.N2()).fill(0, 1, 0, 0);
        final Matrix<N2, N1> B = Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 1);
        final Matrix<N2, N2> C = Matrix.mat(Nat.N2(), Nat.N2()).fill(1, 0, 0, 1);
        final Matrix<N2, N1> D = Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 0);

        {
            // look at how FF works (this is copied from the FF class)
            var discABPair = Discretization.discretizeAB(A, B, dtSeconds);

            Matrix<N2, N2> discA = discABPair.getFirst();
            Matrix<N2, N1> discB = discABPair.getSecond();
            assertEquals(1, discA.get(0, 0), 0.0001);
            assertEquals(0.02, discA.get(0, 1), 0.0001);
            assertEquals(0, discA.get(1, 0), 0.0001);
            assertEquals(1, discA.get(1, 1), 0.0001);
            assertEquals(0.0002, discB.get(0, 0), 0.0001);
            assertEquals(0.02, discB.get(1, 0), 0.0001);
        }

        LinearSystem<N2, N1, N2> plant = new LinearSystem<>(A, B, C, D);
        LinearPlantInversionFeedforward<N2, N1, N2> feedforward = new LinearPlantInversionFeedforward<>(plant,
                dtSeconds);
        // these setpoints correspond to constant acceleration
        // so the feedforward provides the exact solution
        assertEquals(1, feedforward.calculate(VecBuilder.fill(0.0002, 0.02)).get(0, 0), 0.001);
        assertEquals(1, feedforward.calculate(VecBuilder.fill(0.001, 0.04)).get(0, 0), 0.001);
        assertEquals(1, feedforward.calculate(VecBuilder.fill(0.002, 0.06)).get(0, 0), 0.001);
        assertEquals(1, feedforward.calculate(VecBuilder.fill(0.003, 0.08)).get(0, 0), 0.001);
        assertEquals(1, feedforward.calculate(VecBuilder.fill(0.005, 0.1)).get(0, 0), 0.001);

    }
}
