package org.team100.lib.localization;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * If we see multiple tags in the same frame, we can triangulate for better
 * accuracy.
 */
public class TriangulationHelper {

    /**
     * Solve X = T - lambda * d for two targets to obtain X
     * 
     * @param T1 tag translation
     * @param T2 tag translation
     * @param r1 rotation to tag, in camera view
     * @param r2 rotation to tag, in camera view
     * @return translation to intersection
     */
    public static Translation2d solve(
            Translation2d T1,
            Translation2d T2,
            Rotation2d r1,
            Rotation2d r2) {
        // matrix form of the input
        Matrix<N2, N1> t1 = MatBuilder.fill(Nat.N2(), Nat.N1(), T1.getX(), T1.getY());
        Matrix<N2, N1> t2 = MatBuilder.fill(Nat.N2(), Nat.N1(), T2.getX(), T2.getY());
        
        Translation2d T = T2.minus(T1);
        Matrix<N2, N1> t = MatBuilder.fill(Nat.N2(), Nat.N1(), T.getX(), T.getY());
        
        // unit vectors towards the targets
        Translation2d d1 = new Translation2d(1, r1);
        Translation2d d2 = new Translation2d(1, r2);
        
        // solve for range to each target
        Matrix<N2, N2> d = MatBuilder.fill(Nat.N2(), Nat.N2(), -d1.getX(), d2.getX(), -d1.getY(), d2.getY());
        Matrix<N2, N1> lambda = d.solve(t);

        // X = T - lambda*d. take the average of the two targets
        Matrix<N2, N2> D = MatBuilder.fill(Nat.N2(), Nat.N2(), -d1.getX(), -d2.getX(), -d1.getY(), -d2.getY());        
        Matrix<N2, N1> X = D.times(lambda).plus(t1).plus(t2).times(0.5);
        return new Translation2d(X.get(0, 0), X.get(1, 0));
    }

    private TriangulationHelper() {
        //
    }
}
