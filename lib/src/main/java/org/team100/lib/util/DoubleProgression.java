package org.team100.lib.util;

import java.util.Iterator;
import java.util.stream.IntStream;

import edu.wpi.first.math.Pair;

public class DoubleProgression implements Iterable<Double> {
    private final double start;
    private final double step;
    private final int size;

    /**
     * A progression of values of type `Double`.
     */
    public DoubleProgression(
            double start,
            double step,
            int size) {
        this.start = start;
        this.step = step;
        this.size = size;
    }

    public static DoubleProgression fromClosedInterval(double start, double endInclusive, int count) {
        double step = 0;
        if (count == 0) {
            step = 0.0;
        } else if (count == 1) {
            step = 1.0;
        } else {
            step = (endInclusive - start) / (count - 1);
        }
        return new DoubleProgression(start, step, count);
    }

    public DoubleProgression plus(double offset) {
        return new DoubleProgression(start + offset, step, size);
    }

    public DoubleProgression minus(double offset) {
        return new DoubleProgression(start - offset, step, size);
    }

    public DoubleProgression unaryMinus() {
        return new DoubleProgression(-start, -step, size);
    }

    public boolean isEmpty() {
        return size == 0;
    }

    public double rawIndex(double query) {
        return (query - start) / step;
    }

    public int floorIndex(double query) {
        return (int) Math.floor(rawIndex(query));
    }

    public int ceilIndex(double query) {
        return (int) Math.ceil(rawIndex(query));
    }

    public double get(int index) {
        return start + step * index;
    }

    public boolean contains(double query) {
        var rawIndex = rawIndex(query);
        if (rawIndex < 0) {
            return false;
        } else {
            return Math.ceil(rawIndex) < size;
        }
    }

    public int size() {
        return size;
    }

    public Pair<DoubleProgression, DoubleProgression> split(double sep) {
        int sepIndex = ceilIndex(sep);

        if (sepIndex < 0) {
            return new Pair<>(new DoubleProgression(sep, step, 0), this);
        } else if (sepIndex >= size) {
            return new Pair<>(this, new DoubleProgression(sep, step, 0));
        } else {
            return new Pair<>(new DoubleProgression(start, step, sepIndex),
                    new DoubleProgression(get(sepIndex), step, size - sepIndex));
        }

    }

    /**
     * Iterator implementation for [DoubleProgression].
     */
    class IteratorImpl implements Iterator<Double> {
        private final Iterator<Integer> iterator;

        public IteratorImpl() {
            iterator = IntStream.range(0, size - 1).iterator();
        }

        public boolean hasNext() {
            return iterator.hasNext();
        }

        public Double next() {
            return DoubleProgression.this.get(iterator.next());
        }
    }

    public Iterator<Double> iterator() {
        return new IteratorImpl();
    }

    public static DoubleProgression plus(double x, DoubleProgression progression) {
        return progression.plus(x);
    }

    public static DoubleProgression minus(double x, DoubleProgression progression) {
        return new DoubleProgression(x - progression.start, -progression.step, progression.size);
    }

    public double getStart() {
        return start;
    }

    public double getStep() {
        return step;
    }

    public int getSize() {
        return size;
    }

}
