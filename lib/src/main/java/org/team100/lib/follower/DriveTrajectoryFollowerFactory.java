package org.team100.lib.follower;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

/**
 * Known-good controller settings.
 * 
 * joel 20240311 changed ptheta from 2 to 1.3
 */
public class DriveTrajectoryFollowerFactory {

    private final DriveTrajectoryFollowerUtil m_util;

    public DriveTrajectoryFollowerFactory(DriveTrajectoryFollowerUtil util) {
        m_util = util;
    }

    public DriveTrajectoryFollower fancyPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 2.4, 1.3);
    }

    public DriveTrajectoryFollower straightPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 4, 4);
    }

    public DriveTrajectoryFollower newNewPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 5.5, 4);
    }

    public DriveTrajectoryFollower complementPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 6, 6);
    }

    public DriveTrajectoryFollower goodPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 1, 1.3);
    }

    public DriveTrajectoryFollower stageBase(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 2, 1.3);
    }

    public DriveTrajectoryFollower autoPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 1, 1.3);
    }

    public DriveTrajectoryFollower ffOnly(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, true, 2.4, 1.3);
    }

    public static DriveTrajectoryFollower purePursuit(LoggerFactory parent, SwerveKinodynamics swerveKinodynamics) {
        return new DrivePursuitFollower(parent, swerveKinodynamics);
    }

    public static DriveTrajectoryFollower ramsete(LoggerFactory parent) {
        return new DriveRamseteFollower(parent);
    }

    public DriveTrajectoryFollower testPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 2.4, 2.4);
    }

    public DriveTrajectoryFollower testFFOnly(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, true, 2.4, 2.4);
    }

    public DriveTrajectoryFollower fasterCurves(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, true, 4.5, 4.5);
    }
}
