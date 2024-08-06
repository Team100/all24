package org.team100.math;

import edu.wpi.first.math.jni.EigenJNI;

public class RBFInterpolator {
    public RBFInterpolator() {
        // row-major
        double[] A = new double[]{1, 0, 0, 1};
        int Arows = 2;
        int Acols = 2;
        double[] B = new double[]{1, 0, 0, 1};
        int Brows = 2;
        int Bcols = 2;
        double[] dst = new double[4];
        EigenJNI.solveFullPivHouseholderQr(A, Arows, Acols, B, Brows, Bcols, dst);
        System.out.printf("%5.3f %5.3f\n%5.3f %5.3f\n", dst[0],dst[1],dst[2],dst[3]);
    }

}
