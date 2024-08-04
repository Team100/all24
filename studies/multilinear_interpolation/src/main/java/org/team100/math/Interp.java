package org.team100.math;

import edu.wpi.first.wpilibj.Timer;
import smile.interpolation.KrigingInterpolation;
import smile.interpolation.RBFInterpolation;
import smile.math.rbf.GaussianRadialBasis;

public class Interp {

    private static final int kDims = 4;
    private static final int kRows = 1000;

    private final double[][] x;
    private final double[] y;

    public Interp() {
        x = new double[kRows][kDims];
        y = new double[kRows];
        for (int i = 0; i < kRows; i++) {
            for (int j = 0; j < kDims; j++) {
                x[i][j] = (double)i + j;
            }
            y[i] = 0.001 * i;
        }
    }

    public void kriging() {
        KrigingInterpolation kriging = new KrigingInterpolation(x, y);
        double start = Timer.getFPGATimestamp();
        double[] observation = new double[] { 1.5, 2.5, 3.5, 4.5 };
        int n = 100000;
        for (int i = 0; i < n; ++i) {
            kriging.interpolate(observation);
        }
        double end = Timer.getFPGATimestamp();
        double duration = end - start;
        System.out.printf("Kriging duration (s) %5.3f\n", duration);
        System.out.printf("Kriging duation per prediction (us) %5.3f\n", duration * 1e6 / n);
    }

    public void rbf() {
        RBFInterpolation rbf = new RBFInterpolation(x, y, new GaussianRadialBasis());
        double start = Timer.getFPGATimestamp();
        double[] observation = new double[] { 1.5, 2.5, 3.5, 4.5 };
        int n = 100000;
        for (int i = 0; i < n; ++i) {
            rbf.interpolate(observation);
        }
        double end = Timer.getFPGATimestamp();
        double duration = end - start;
        System.out.printf("RBF duration (s) %5.3f\n", duration);
        System.out.printf("RBF duation per prediction (us) %5.3f\n", duration * 1e6 / n);
    }

}
