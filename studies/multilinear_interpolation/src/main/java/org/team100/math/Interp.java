package org.team100.math;

import java.awt.event.WindowAdapter;
import java.lang.reflect.InvocationTargetException;
import java.util.concurrent.CountDownLatch;

import javax.swing.JFrame;

import edu.wpi.first.wpilibj.Timer;
import smile.interpolation.KrigingInterpolation;
import smile.interpolation.LinearInterpolation;
import smile.interpolation.RBFInterpolation;
import smile.math.rbf.GaussianRadialBasis;
import smile.plot.swing.*;

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
                x[i][j] = (double) i + j;
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

    public void demo() throws InvocationTargetException, InterruptedException {
        double[] x = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };
        double[] y = { 0.0, 0.8415, 0.9093, 0.1411, -0.7568, -0.9589, -0.2794 };
        double[][] points = new double[x.length][2];
        for (int i = 0; i < x.length; i++) {
            points[i][0] = x[i];
            points[i][1] = y[i];
        }

        var linear = new LinearInterpolation(x, y);
        double[][] data = new double[61][2];
        for (int i = 0; i < data.length; i++) {
            data[i][0] = i * 0.1;
            data[i][1] = linear.interpolate(data[i][0]);
        }

        Canvas canvas = ScatterPlot.of(points).canvas();
        canvas.add(LinePlot.of(data));
        JFrame frame = canvas.window();

        // Waits for plot window to be closed.
        CountDownLatch latch = new CountDownLatch(1);
        frame.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(java.awt.event.WindowEvent windowEvent) {
                latch.countDown();
            }
        });
        latch.await();
    }

}
