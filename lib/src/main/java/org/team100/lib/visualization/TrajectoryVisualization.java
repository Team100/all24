package org.team100.lib.visualization;

import java.util.List;

import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPoint;

import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class TrajectoryVisualization {
    private static final String kTrajectory = "trajectory";

    private final SupplierLogger m_fieldLogger;

    public TrajectoryVisualization(SupplierLogger fieldLogger) {
        m_fieldLogger = fieldLogger;
    }

    public void setViz(Trajectory100 m_trajectory) {
        m_fieldLogger.logDoubleArray(Level.TRACE, kTrajectory, () -> fromTrajectory100(m_trajectory));
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
        m_fieldLogger.logDoubleArray(Level.TRACE, kTrajectory, () -> fromWPITrajectory(m_trajectory));
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
        m_fieldLogger.logDoubleArray(Level.TRACE, kTrajectory, () -> fromPoses(poses));
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
        m_fieldLogger.logDoubleArray(Level.TRACE, kTrajectory, () -> new double[0]);
    }

}
