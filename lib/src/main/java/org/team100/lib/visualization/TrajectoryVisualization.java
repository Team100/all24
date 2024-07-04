package org.team100.lib.visualization;

import java.util.List;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPoint;

import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class TrajectoryVisualization {
    private static final String kTrajectory = "trajectory";
    private static final Telemetry.Logger fieldLogger = Telemetry.get().fieldLogger();

    private TrajectoryVisualization() {
        //
    }

    public static void setViz(Trajectory100 m_trajectory) {
        double[] arr = new double[m_trajectory.length() * 3];
        int ndx = 0;
        for (TrajectoryPoint p : m_trajectory.getPoints()) {
            Pose2d pose = p.state().state().getPose();
            arr[ndx + 0] = pose.getTranslation().getX();
            arr[ndx + 1] = pose.getTranslation().getY();
            arr[ndx + 2] = pose.getRotation().getDegrees();
            ndx += 3;
        }
        fieldLogger.log(Level.DEBUG, kTrajectory, arr);
    }

    public static void setViz(Trajectory m_trajectory) {
        double[] arr = new double[m_trajectory.getStates().size() * 3];
        int ndx = 0;
        for (State p : m_trajectory.getStates()) {
            Pose2d pose = p.poseMeters;
            arr[ndx + 0] = pose.getTranslation().getX();
            arr[ndx + 1] = pose.getTranslation().getY();
            arr[ndx + 2] = pose.getRotation().getDegrees();
            ndx += 3;
        }
        fieldLogger.log(Level.DEBUG, kTrajectory, arr);
    }

    public static void setViz(List<Pose2d> poses) {
        double[] arr = new double[poses.size() * 3];
        int ndx = 0;
        for (Pose2d pose : poses) {
            arr[ndx + 0] = pose.getTranslation().getX();
            arr[ndx + 1] = pose.getTranslation().getY();
            arr[ndx + 2] = pose.getRotation().getDegrees();
            ndx += 3;
        }
        fieldLogger.log(Level.DEBUG, kTrajectory, arr);
    }

    public static void setViz(ChoreoTrajectory trajectory) {
        setViz(List.of(trajectory.getPoses()));
    }

    public static void clear() {
        fieldLogger.log(Level.DEBUG, kTrajectory, new double[0]);
    }

}
