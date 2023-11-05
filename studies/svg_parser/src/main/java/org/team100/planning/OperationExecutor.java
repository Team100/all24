package org.team100.planning;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.geom.Ellipse2D;

import javax.swing.JFrame;
import javax.swing.WindowConstants;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import edu.wpi.first.math.trajectory.Trajectory;

/** Execute the list of operations one at a time */
public class OperationExecutor {
    private final XYSeriesCollection dataset = new XYSeriesCollection();
    private final XYPlot xy;

    private int seriesIdx = 0;

    public OperationExecutor() {
        JFrame frame = new JFrame("Chart Collection");
        JFreeChart chart = ChartFactory.createScatterPlot("Plotter", "X", "Y", dataset);
        chart.removeLegend();
        xy = (XYPlot) chart.getPlot();
        xy.setBackgroundPaint(Color.WHITE);
        ChartPanel panel = new ChartPanel(chart) {
            @Override
            public Dimension getPreferredSize() {
                return new Dimension(500, 500);
            }
        };
        frame.add(panel, BorderLayout.EAST);
        frame.setLocationRelativeTo(null);
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        frame.pack();
        frame.setVisible(true);
    }

    public void executeTrajectory(boolean penDown, Trajectory trajectory, double dtSec) {
        Color color = penDown ? Color.BLACK : Color.GREEN;

        XYSeries series1 = new XYSeries(String.format("trajectory %d", seriesIdx));
        for (double t = 0; t < trajectory.getTotalTimeSeconds(); t += dtSec) {
            Trajectory.State s = trajectory.sample(t);
            series1.add(s.poseMeters.getX(), s.poseMeters.getY());
        }

        dataset.addSeries(series1);
        xy.getRenderer().setSeriesShape(seriesIdx, new Ellipse2D.Double(0, 0, 5, 5));
        xy.getRenderer().setSeriesPaint(seriesIdx, color);
        seriesIdx++;
        fixScales();
    }

    /** Make the x and y scales the same, and autorange. */
    private void fixScales() {
        xy.getDomainAxis().setAutoRange(true);
        xy.getRangeAxis().setAutoRange(true);
        double domainLength = xy.getDomainAxis().getRange().getLength();
        double rangeLength = xy.getRangeAxis().getRange().getLength();
        if (domainLength > rangeLength) {
            double rangeCenter = xy.getRangeAxis().getRange().getCentralValue();
            xy.getRangeAxis().setRange(rangeCenter - domainLength / 2, rangeCenter + domainLength / 2);
        } else {
            double domainCenter = xy.getDomainAxis().getRange().getCentralValue();
            xy.getDomainAxis().setRange(domainCenter - rangeLength / 2, domainCenter + rangeLength / 2);
        }
    }

}
