package org.team100.lib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/** Example of handling non-euclidean metrics; this has an angle in row zero. */
public class AngularRandomVector<States extends Num> extends RandomVector<States> {

    public AngularRandomVector(Matrix<States, N1> x, Variance<States> P) {
        super(x, P);
        x.set(0, 0, MathUtil.angleModulus(x.get(0, 0)));
    }

    public AngularRandomVector<States> make(Matrix<States, N1> x, Variance<States> P) {
        return new AngularRandomVector<>(x, P);
    }

    @Override
    public RandomVector<States> plus(RandomVector<States> other) {
        RandomVector<States> x = super.plus(other);
        x.x.set(0, 0, MathUtil.angleModulus(x.x.get(0, 0)));
        return x;
    }

    @Override
    public RandomVector<States> minus(RandomVector<States> other) {
        RandomVector<States> x = super.minus(other);
        x.x.set(0, 0, MathUtil.angleModulus(x.x.get(0, 0)));
        return x;
    }

    @Override
    public Matrix<States, N1> xplus(Matrix<States, N1> otherx) {
        Matrix<States, N1> x = super.xplus(otherx);
        x.set(0, 0, MathUtil.angleModulus(x.get(0, 0)));
        return x;
    }

    @Override
    public Matrix<States, N1> xminus(Matrix<States, N1> other) {
        Matrix<States, N1> x = super.xminus(other);
        x.set(0, 0, MathUtil.angleModulus(x.get(0, 0)));
        return x;
    }
    @Override
    public String toString() {
        return "AngularRandomVector [x=" + x + ", P=" + Kxx + "]";
    }
}
