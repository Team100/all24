package org.team100.lib.controller;

import org.team100.lib.math.Jacobian;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.DARE;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.Discretization;

/**
 * Fixed gain linearized around zero, cribbed from LQR.
 */
public class GainCalculator<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final Matrix<Inputs, States> m_K;

    /**
     * Calculate LQR gain linearized around zero. I don't think it's worth the
     * trouble to make K sensitive to x, since all our systems are well behaved.
     * 
     * This is all cribbed from WPI code, I don't really understand what it's doing.
     * 
     * @param dtSeconds time step for discretization. choose a value that's
     *                  convenient, e.g. the robot loop period.
     */
    public GainCalculator(
            NonlinearPlant<States, Inputs, Outputs> plant,
            Vector<States> qelms,
            Vector<Inputs> relms,
            double dtSeconds) {
        Matrix<Inputs, N1> kUZero = new Matrix<>(plant.inputs(), Nat.N1());
        Matrix<States, States> m_Q = StateSpaceUtil.makeCostMatrix(qelms);
        Matrix<Inputs, Inputs> m_R = StateSpaceUtil.makeCostMatrix(relms);
        RandomVector<States> x = new RandomVector<>(new Matrix<>(plant.states(), Nat.N1()),
                Variance.zero(plant.states()));
        Matrix<States, States> A = Jacobian.numericalJacobianX(plant.states(), plant.states(), plant::f,
                x, kUZero);
        Matrix<States, Inputs> B = Jacobian.numericalJacobianU(plant.states(), plant.inputs(), plant::f,
                x, kUZero);

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

        var S = DARE.dare(discA, discB, m_Q, m_R);

        // K = (BᵀSB + R)⁻¹BᵀSA
        m_K = discB
                .transpose()
                .times(S)
                .times(discB)
                .plus(m_R)
                .solve(discB.transpose().times(S).times(discA));
    }

    public Matrix<Inputs, States> getK() {
        return m_K;
    }
}
