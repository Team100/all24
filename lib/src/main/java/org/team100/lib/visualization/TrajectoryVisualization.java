package org.team100.lib.visualization;

import java.util.List;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleArraySupplierLogger2;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPoint;

import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class TrajectoryVisualization {
    private static final String kTrajectory = "trajectory";

    private final SupplierLogger2 m_fieldLogger;

    private final DoubleArraySupplierLogger2 m_log_trajectory;

    public TrajectoryVisualization(SupplierLogger2 fieldLogger) {
        m_fieldLogger = fieldLogger;
        m_log_trajectory = m_fieldLogger.doubleArrayLogger(Level.TRACE, kTrajectory);
    }

    public void setViz(Trajectory100 m_trajectory) {
        m_log_trajectory.log( () -> fromTrajectory100(m_trajectory));
    }

    private static double[] fromTrajectory100(Trajectory100 m_trajectory) {
        double[] arr = new double[m_trajectory.length() * 3];
        int ndx = 0;
        for (TrajectoryPoint p : m_trajectory.getPoints()) {
            Pose2d pose = p.state().state().getPose();
            arr[ndx + 0] = pose.getTranslation().getX();
            arr[ndx + 1] = pose.getTranslation().getY();
            arr[ndx + 2] = pose.getRotation().getDegrees();
            ndx += 3;
        }
        return arr;
    }

    public void setViz(Trajectory m_trajectory) {
        m_log_trajectory.log( () -> fromWPITrajectory(m_trajectory));
    }

    private static double[] fromWPITrajectory(Trajectory m_trajectory) {
        double[] arr = new double[m_trajectory.getStates().size() * 3];
        int ndx = 0;
        for (State p : m_trajectory.getStates()) {
            Pose2d pose = p.poseMeters;
            arr[ndx + 0] = pose.getTranslation().getX();
            arr[ndx + 1] = pose.getTranslation().getY();
            arr[ndx + 2] = pose.getRotation().getDegrees();
            ndx += 3;
        }
        return arr;
    }

    public void setViz(List<Pose2d> poses) {
        m_log_trajectory.log( () -> fromPoses(poses));
    }

    private static double[] fromPoses(List<Pose2d> poses) {
        double[] arr = new double[poses.size() * 3];
        int ndx = 0;
        for (Pose2d pose : poses) {
            arr[ndx + 0] = pose.getTranslation().getX();
            arr[ndx + 1] = pose.getTranslation().getY();
            arr[ndx + 2] = pose.getRotation().getDegrees();
            ndx += 3;
        }
        return arr;
    }

    public void setViz(ChoreoTrajectory trajectory) {
        setViz(List.of(trajectory.getPoses()));
    }

    public void clear() {
        m_log_trajectory.log( () -> new double[0]);
    }

}
