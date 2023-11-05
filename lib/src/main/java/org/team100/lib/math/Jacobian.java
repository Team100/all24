package org.team100.lib.math;

import java.util.function.BiFunction;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Copy of the WPI version but for random variables.
 */
public class Jacobian {
    private static final double kEpsilon = 1e-5;

    // note this is used for both f and h
    // TODO make a separate version for h
    public static <Rows extends Num, States extends Num, Inputs extends Num, Outputs extends Num> Matrix<Rows, States> numericalJacobianX(
            Nat<Rows> rows,
            Nat<States> states,
            BiFunction<RandomVector<States>, Matrix<Inputs, N1>, RandomVector<Outputs>> f,
            RandomVector<States> x,
            Matrix<Inputs, N1> u) {
        Matrix<Rows, States> result = new Matrix<>(rows, states);
        for (int i = 0; i < states.getNum(); i++) {
            RandomVector<States> dxPlus = x.copy();
            RandomVector<States> dxMinus = x.copy();
            dxPlus.x.set(i, 0, dxPlus.x.get(i, 0) + kEpsilon);
            dxMinus.x.set(i, 0, dxMinus.x.get(i, 0) - kEpsilon);
            var dF = f.apply(dxPlus, u).x.minus(f.apply(dxMinus, u).x).div(2 * kEpsilon);
            result.setColumn(i, Matrix.changeBoundsUnchecked(dF));
        }
        return result;
    }

    public static <Rows extends Num, States extends Num, Inputs extends Num> Matrix<Rows, Inputs> numericalJacobianU(
            Nat<Rows> rows,
            Nat<Inputs> inputs,
            BiFunction<RandomVector<States>, Matrix<Inputs, N1>, RandomVector<States>> f,
            RandomVector<States> x,
            Matrix<Inputs, N1> u) {
        Matrix<Rows, Inputs> result = new Matrix<>(rows, inputs);
        for (int i = 0; i < inputs.getNum(); i++) {
            Matrix<Inputs, N1> duPlus = u.copy();
            Matrix<Inputs, N1> duMinus = u.copy();
            duPlus.set(i, 0, duPlus.get(i, 0) + kEpsilon);
            duMinus.set(i, 0, duMinus.get(i, 0) - kEpsilon);
            Matrix<States, N1> dF = f.apply(x, duPlus).x.minus(f.apply(x, duMinus).x).div(2 * kEpsilon);
            result.setColumn(i, Matrix.changeBoundsUnchecked(dF));
        }
        return result;
    }

    private Jacobian() {}
}
