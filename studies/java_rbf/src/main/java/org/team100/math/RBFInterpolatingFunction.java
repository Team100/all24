package org.team100.math;

import java.util.function.Function;

public class RBFInterpolatingFunction<T, U> implements Function<T, U> {
    abstract class Adapter<V> {
        abstract double[] toArray(V val);
        abstract int size();
    }

    public RBFInterpolatingFunction(Adapter<T> xadapter, Adapter<U> yadapter) {

    }

    @Override
    public U apply(T t) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'apply'");
    }

}
