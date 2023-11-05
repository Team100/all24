package org.team100.lib.controller;

import org.team100.lib.config.Identity;
import org.team100.lib.motion.drivetrain.SpeedLimits;

import edu.wpi.first.math.controller.PIDController;

public class DriveControllersFactory {
    public static class Config {
        public PidGains thetaGain = new PidGains(3.0, 0, 0, 0.01, 0.01, true);
        public PidGains compBotCartesianGain = new PidGains(3, 2, 0, 0.1, 0.01, false);
        public PidGains swerveOneCartesianGain = new PidGains(0.15, 0, 0, 0.1, 0.2, false);
        // public PidGains swerveOneCartesianGain = new PidGains(2, 0.1, 0.15, 0.1, 0.01, false);

        public PidGains swerveTwoCartesianGain = new PidGains(2.0, 0.1, 0.15, 0.1, 0.01, false);
    }

    private final Config m_config = new Config();

    public DriveControllers get(Identity identity, SpeedLimits speedLimits) {
        // switch (identity) {
        //     case COMP_BOT:
        //         return new DriveControllers(
        //                 pid(m_config.compBotCartesianGain),
        //                 pid(m_config.compBotCartesianGain),
        //                 pid(m_config.thetaGain));
        //     case SWERVE_ONE:
        //         return new DriveControllers(
        //                 pid(m_config.swerveOneCartesianGain),
        //                 pid(m_config.swerveOneCartesianGain),
        //                 pid(m_config.thetaGain));
        //     case SWERVE_TWO:
        //         return new DriveControllers(
        //                 pid(m_config.swerveTwoCartesianGain),
        //                 pid(m_config.swerveTwoCartesianGain),
        //                 pid(m_config.thetaGain));
        //     default:
        //         // these RoboRIO's are have no drivetrains
        //         return new DriveControllers(
        //                 new PIDController(1, 0.0, 0.0),
        //                 new PIDController(1, 0.0, 0.0),
        //                 new PIDController(1, 0.0, 0.0));
        // }

        
        return new DriveControllers(
            pid(m_config.compBotCartesianGain),
            pid(m_config.compBotCartesianGain),
            pid(m_config.thetaGain));
    }

    private static PIDController pid(PidGains g) {
        PIDController pid = new PIDController(g.p, g.i, g.d);
        pid.setIntegratorRange(-1.0 * g.integratorRange, g.integratorRange);
        pid.setTolerance(g.tolerance);
        if (g.continuous)
            pid.enableContinuousInput(-1.0 * Math.PI, Math.PI);
        return pid;
    }
}
