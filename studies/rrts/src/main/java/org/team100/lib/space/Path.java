package org.team100.lib.space;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;


public class Path<States extends Num> implements Comparable<Path<States>> {

    /** The total length of the computed path     */
    private final double distance;

    /** The states along the path.  It's two lists so i can see where the join is. */
    private final List<Matrix<States, N1>> states_A;
    private final List<Matrix<States, N1>> states_B;

    public Path(double distance, List<Matrix<States, N1>> states_A, List<Matrix<States, N1>> states_B) {
        this.distance = distance;
        this.states_A = states_A;
        this.states_B = states_B;
    }

    /**
     * Paths are comparable by their distances.
     *
     * @param that the path to compare to
     * @return -1 if this path is shorter, 0 is equal in length, 1 if longer
     * than the argument path.
     */
    @Override
    public int compareTo(Path<States> that) {
        return Double.compare(distance, that.distance);
    }

    public static <States extends Num> boolean isBetter(Path<States> a, Path<States> b) {
        if (a == null) {
            return false;
        }
        if (b == null) {
            return true;
        }
        return a.distance < b.distance;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Path<?>)) return false;

        // wildcard here since we don't actually care what the parameter is
        Path<?> path = (Path<?>) o;

        if (Double.compare(path.distance, distance) != 0) return false;
        return statesEqual(states_A, path.states_A) && statesEqual(states_B, path.states_B);
    }

    // wildcard type here accommodates the cast above
    private boolean statesEqual(List<? extends Matrix<?,?>> states, List<? extends Matrix<?, ?>> otherstates) {
        if (states.size() != otherstates.size()) return false;
        Iterator<? extends Matrix<?, ?>> i1 = states.iterator();
        Iterator<? extends Matrix<?, ?>> i2 = otherstates.iterator();
        while (i1.hasNext()) {
            if (! i1.next().equals(i2.next())) {
                 return false;
            }
        }
        return true;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = distance != +0.0d ? Double.doubleToLongBits(distance) : 0L;
        result = (int) (temp ^ (temp >>> 32));
        for (Matrix<States, N1> config : states_A) {
            result = 31 * result + config.hashCode();
        }
        for (Matrix<States, N1> config : states_B) {
            result = 31 * result + config.hashCode();
        }
        return result;
    }

    public double getDistance() {
        return distance;
    }

    public List<Matrix<States, N1>> getStatesA() {
        List<Matrix<States, N1>> allStates = new ArrayList<>();
        allStates.addAll(states_A);
        return allStates;
    }
    
    public List<Matrix<States, N1>> getStatesB() {
        List<Matrix<States, N1>> allStates = new ArrayList<>();
        allStates.addAll(states_B);
        return allStates;
    }

    @Override
    public String toString() {
        String result="";
        result += "Path [_dist=" + String.format("%8.5f", distance);
        result += " states_A=[\n";
        for (Matrix<States, N1> d : states_A) {
            result += d.toString() + "\n";
        } 
        result += "]\n";
        result += " states_B=[\n";
        for (Matrix<States, N1> d : states_B) {
            result += d.toString() + "\n";
        } 
        result += "]]";
    return result;
    }
}
