package org.team100.lib.space;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * A point in some metric space, not necessarily Euclidean; the KD Tree assumes
 * Euclidean metric, though, so, like, watch out.
 */
public interface Point<States extends Num> {
    /** The vector that describes this point. */
    Matrix<States, N1> getState();
}
