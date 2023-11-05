package org.team100.lib.system;

import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * to make testing easier
 * 
 * @param <States>  states
 * @param <Inputs>  inputs
 * @param <Outputs> outputs
 */
public class MockNonlinearPlant<States extends Num, Inputs extends Num, Outputs extends Num>
        implements NonlinearPlant<States, Inputs, Outputs> {

    @Override
    public RandomVector<States> f(RandomVector<States> x, Matrix<Inputs, N1> u) {
        throw new UnsupportedOperationException("Unimplemented method 'f'");
    }

    @Override
    public Matrix<Inputs, N1> finvWrtU(RandomVector<States> x, RandomVector<States> xdot) {
        throw new UnsupportedOperationException("Unimplemented method 'finv'");
    }

    @Override
    public RandomVector<States> finvWrtX(RandomVector<States> xdot, Matrix<Inputs, N1> u) {
        throw new UnsupportedOperationException("Unimplemented method 'finv'");
    }

    @Override
    public RandomVector<Outputs> h(RandomVector<States> x, Matrix<Inputs, N1> u) {
        throw new UnsupportedOperationException("Unimplemented method 'h'");
    }

    @Override
    public RandomVector<States> hinv(RandomVector<Outputs> y, Matrix<Inputs, N1> u) {
        throw new UnsupportedOperationException("Unimplemented method 'hinv'");
    }

    @Override
    public WhiteNoiseVector<States> w() {
        throw new UnsupportedOperationException("Unimplemented method 'xi'");
    }

    @Override
    public Matrix<Inputs, N1> limit(Matrix<Inputs, N1> u) {
        throw new UnsupportedOperationException("Unimplemented method 'limit'");
    }

    @Override
    public Nat<States> states() {
        throw new UnsupportedOperationException("Unimplemented method 'states'");
    }

    @Override
    public Nat<Inputs> inputs() {
        throw new UnsupportedOperationException("Unimplemented method 'inputs'");
    }

    @Override
    public Nat<Outputs> outputs() {
        throw new UnsupportedOperationException("Unimplemented method 'outputs'");
    }

    @Override
    public RandomVector<States> make(Matrix<States, N1> x, Variance<States> P) {
        throw new UnsupportedOperationException("Unimplemented method 'make'");
    }



}