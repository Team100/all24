package org.team100.lib.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.DARE;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;

class KTest {
    static final double kDelta = 0.001;
    static final double kDt = 0.02;

    /**
     * exactly from the LQR constructor, for comparison
     */
    private static Matrix<N1, N2> calculateK(
            Matrix<N2, N2> A,
            Matrix<N2, N1> B,
            Matrix<N2, N2> Q,
            Matrix<N1, N1> R,
            double dtSeconds) {
        var discABPair = Discretization.discretizeAB(A, B, dtSeconds);
        var discA = discABPair.getFirst();
        var discB = discABPair.getSecond();

        if (!StateSpaceUtil.isStabilizable(discA, discB)) {
            var builder = new StringBuilder("The system passed to the LQR is uncontrollable!\n\nA =\n");
            builder.append(discA.getStorage().toString())
                    .append("\nB =\n")
                    .append(discB.getStorage().toString())
                    .append('\n');
            throw new IllegalArgumentException(builder.toString());
        }

        var S = DARE.dare(discA, discB, Q, R);

        // K = (BᵀSB + R)⁻¹BᵀSA
        Matrix<N1, N2> m_K = discB
                .transpose()
                .times(S)
                .times(discB)
                .plus(R)
                .solve(discB.transpose().times(S).times(discA));
        return m_K;
    }

    /**
     * verify that we're calculating K the same as LQR does.
     */
    @Test
    void testK() {
        Nat<N2> states = Nat.N2();
        Nat<N1> inputs = Nat.N1();
        Matrix<N2, N2> A = Matrix.mat(states, states).fill(0, 1, 0, 0);
        Matrix<N2, N1> B = Matrix.mat(states, inputs).fill(0, 1);
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        Matrix<N2, N2> Q = StateSpaceUtil.makeCostMatrix(stateTolerance);
        Matrix<N1, N1> R = StateSpaceUtil.makeCostMatrix(controlTolerance);
        Matrix<N1, N2> K = calculateK(A, B, Q, R, kDt);
        assertEquals(572.773, K.get(0, 0), kDelta);
        assertEquals(44.336, K.get(0, 1), kDelta);
    }

}
