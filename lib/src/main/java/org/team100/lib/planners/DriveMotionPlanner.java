package org.team100.lib.planners;

import org.team100.lib.controller.DriveFeedforwardController;
import org.team100.lib.controller.DrivePIDController;
import org.team100.lib.controller.DrivePursuitController;
import org.team100.lib.controller.DriveRamseteController;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.util.MathUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** This is derived from 254's 2023 version */
public class DriveMotionPlanner {

    public static final Telemetry t = Telemetry.get();

    public enum FollowerType {
        FEEDFORWARD_ONLY,
        PID,
        PURE_PURSUIT,
        RAMSETE
    }

    FollowerType mFollowerType = FollowerType.PID;

    public void setFollowerType(FollowerType type) {
        mFollowerType = type;
    }

    private final DriveRamseteController m_ramsete = new DriveRamseteController();
    private final DrivePIDController m_pid = new DrivePIDController();
    private final DrivePursuitController m_pursuit = new DrivePursuitController();
    private final DriveFeedforwardController m_ff = new DriveFeedforwardController();

    public void setTrajectory(final TrajectoryTimeIterator trajectory) {
        if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
            m_ff.setTrajectory(trajectory);
        }
        if (mFollowerType == FollowerType.RAMSETE) {
            m_ramsete.setTrajectory(trajectory);
        }
        if (mFollowerType == FollowerType.PID) {
            m_pid.setTrajectory(trajectory);
        }
        if (mFollowerType == FollowerType.PURE_PURSUIT) {
            m_pursuit.setTrajectory(trajectory);
        }

    }

    public void reset() {
        m_pursuit.reset();
        m_ff.reset();
        m_pid.reset();
        m_ramsete.reset();
    }

    public ChassisSpeeds update(double timestamp, Pose2d current_state, Twist2d current_velocity) {

        if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
            return m_ff.updateFeedforward(timestamp, current_state);
        }
        if (mFollowerType == FollowerType.RAMSETE) {
            return m_ramsete.updateRamsete(timestamp, current_state, current_velocity);
        }
        if (mFollowerType == FollowerType.PID) {
            return m_pid.updatePIDChassis(timestamp, current_state);
        }
        if (mFollowerType == FollowerType.PURE_PURSUIT) {
            return m_pursuit.updatePurePursuit(timestamp, current_state, 0.0);
        }

        return new ChassisSpeeds();
    }

    public boolean isDone() {
        if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
            return m_ff.isDone();
        }
        if (mFollowerType == FollowerType.RAMSETE) {
            return m_ramsete.isDone();
        }
        if (mFollowerType == FollowerType.PID) {
            return m_pid.isDone();
        }
        if (mFollowerType == FollowerType.PURE_PURSUIT) {
            return m_pursuit.isDone();
        }
        return true;
    }

    private Pose2d getError() {
        if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
            return m_ff.getError();
        }
        if (mFollowerType == FollowerType.RAMSETE) {
            return m_ramsete.getError();
        }
        if (mFollowerType == FollowerType.PID) {
            return m_pid.getError();
        }
        if (mFollowerType == FollowerType.PURE_PURSUIT) {
            return m_pursuit.getError();
        }
        return new Pose2d();
    }

    public synchronized Translation2d getTranslationalError() {
        return new Translation2d(
                getError().getTranslation().getX(),
                getError().getTranslation().getY());
    }

    public synchronized Rotation2d getHeadingError() {
        return getError().getRotation();
    }

    public synchronized TimedPose getSetpoint() {

        if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
            return m_ff.mSetpoint;
        }
        if (mFollowerType == FollowerType.RAMSETE) {
            return m_ramsete.mSetpoint;
        }
        if (mFollowerType == FollowerType.PID) {
            return m_pid.mSetpoint;
        }
        if (mFollowerType == FollowerType.PURE_PURSUIT) {
            return m_pursuit.mSetpoint;
        }

        return null;
    }
}
