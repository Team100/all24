package frc;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.arm.ArmTrajectories;
import frc.robot.armMotion.ArmAngles;
import frc.robot.armMotion.ArmKinematics;

/**
 * Visualize trajectories in joint space. Click "Run" below in vscode to see it.
 */
public class JointVisualizer {

    /** Return a dataset with one series with proximal in x and distal in y */
    private static XYSeriesCollection joints() {
        ArmKinematics kinematics = new ArmKinematics(.93, .92);
        XYSeriesCollection dataset = new XYSeriesCollection();
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        ArmTrajectories trajectories = new ArmTrajectories(config);
        Translation2d t0 = kinematics.forward(new ArmAngles(-0.639248, 1.838205)); // safe
        Translation2d mid = kinematics.forward(new ArmAngles(0.089803, 1.681915)); // mid
        Translation2d t1 = new Translation2d(.6, .6);
        Translation2d t2 = new Translation2d(1, .6);
        Translation2d t3 = new Translation2d(1, 1);
        Translation2d t4 = new Translation2d(.6, 1);
        Trajectory trajectorystart = trajectories.makeTrajectory(t0, t1);
        Trajectory trajectory = trajectories.makeTrajectory(t1, t2);
        Trajectory trajectory2 = trajectories.makeTrajectory(t2, t3);
        Trajectory trajectory3 = trajectories.makeTrajectory(t3, t4);
        Trajectory trajectory4 = trajectories.makeTrajectory(t4, t1);
        XYSeries series1 = new XYSeries("Cartesian End");
        XYSeries elbow = new XYSeries("Cartesian Elbow");
        for (double t = 0; t < trajectorystart.getTotalTimeSeconds(); t += 0.05) {
            Trajectory.State s = trajectorystart.sample(t);
            double x1 = s.poseMeters.getX(); // proximal
            double y1 = s.poseMeters.getY(); // distal
            series1.add(x1, y1);
        }
        for (double t = 0; t < trajectory.getTotalTimeSeconds(); t += 0.05) {
            Trajectory.State s = trajectory.sample(t);
            double x1 = s.poseMeters.getX(); // proximal
            double y1 = s.poseMeters.getY(); // distal
            series1.add(x1, y1);
        }
        for (double t = 0; t < trajectory2.getTotalTimeSeconds(); t += 0.05) {
            Trajectory.State s = trajectory2.sample(t);
            double x1 = s.poseMeters.getX(); // proximal
            double y1 = s.poseMeters.getY(); // distal
            series1.add(x1, y1);
        }
        for (double t = 0; t < trajectory3.getTotalTimeSeconds(); t += 0.05) {
            
            Trajectory.State s = trajectory3.sample(t);
            double x1 = s.poseMeters.getX(); // proximal
            double y1 = s.poseMeters.getY(); // distal
            series1.add(x1, y1);
        }
        for (double t = 0; t < trajectory4.getTotalTimeSeconds(); t += 0.05) {
            Trajectory.State s = trajectory4.sample(t);
            double x1 = s.poseMeters.getX(); // proximal
            double y1 = s.poseMeters.getY(); // distal
            series1.add(x1, y1);
        }
        dataset.addSeries(series1);
        XYSeries joints = dataset.getSeries(0);
        int ct = joints.getItemCount();
        for (int i = 0; i < ct; ++i) {
            Number nx = joints.getX(i); // proximal
            Number ny = joints.getY(i); // distal
            double proximal = nx.doubleValue();
            double distal = ny.doubleValue();
            Translation2d a = new Translation2d(proximal, distal);
            ArmAngles c = kinematics.inverse(a);
            Translation2d el = kinematics.elbow(c);
            elbow.add(el.getY(), el.getX());
        }
        dataset.addSeries(elbow);
        XYSeries base = new XYSeries("Base");
        base.add(0, 0);
        dataset.addSeries(base);
        return dataset;
    }

    /**
     * Add end and elbow series.
     * 
     * X represents "z" i.e. height
     * Y represents "x" i.e. forward
     * 
     * @param data has proximal in x and distal in y
     * 
     */
    private static XYSeriesCollection cartesianToJoint(XYSeriesCollection data) {
        ArmKinematics k = new ArmKinematics(.93, .92);
        XYSeries joints = data.getSeries(0);
        XYSeries end = new XYSeries("Joints");
        int ct = joints.getItemCount();
        for (int i = 0; i < ct; ++i) {
            Number nx = joints.getX(i); // proximal
            Number ny = joints.getY(i); // distal
            double proximal = nx.doubleValue();
            double distal = ny.doubleValue();
            Translation2d a = new Translation2d(proximal, distal);
            ArmAngles c = k.inverse(a);
            end.add(c.th1, c.th2);
        }
        XYSeriesCollection result = new XYSeriesCollection();
        result.addSeries(end);
        return result;

    }

    /** Click "Run" below in vscode. */
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {

            JFrame frame = new JFrame("Chart Collection");

            XYSeriesCollection translation = joints();
            JFreeChart jointChart = ChartFactory.createScatterPlot(
                    "Trajectory in Cartesian Space",
                    "X (Left/Right)",
                    "Y (Up/Down)",
                    translation);

            XYPlot cartesianXY = (XYPlot) jointChart.getPlot();
            cartesianXY.setBackgroundPaint(Color.WHITE);
            cartesianXY.getDomainAxis().setRange(-1, 1.5); // 2
            cartesianXY.getRangeAxis().setRange(0, 2.5); // 2

            ChartPanel jointPanel = new ChartPanel(jointChart) {
                @Override
                public Dimension getPreferredSize() {
                    return new Dimension(400, 400);
                }
            };
            frame.add(jointPanel, BorderLayout.EAST);

            XYSeriesCollection cartesian = cartesianToJoint(translation);
            JFreeChart cartesianChart = ChartFactory.createScatterPlot(
                    "Trajectory in Joint Space",
                    "Lower",
                    "Upper",
                    cartesian);

            XYPlot jointXY = (XYPlot) cartesianChart.getPlot();
            jointXY.setBackgroundPaint(Color.WHITE);
            jointXY.getDomainAxis().setRange(-0.7, 0.5); // 2.5
            jointXY.getRangeAxis().setRange(0, 3.14);

            ChartPanel cartesianPanel = new ChartPanel(cartesianChart) {
                @Override
                public Dimension getPreferredSize() {
                    return new Dimension(400, 400);
                }
            };
            frame.add(cartesianPanel, BorderLayout.WEST);
            frame.setLocationRelativeTo(null);
            frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
            frame.pack();
            frame.setVisible(true);
        });
    }
}