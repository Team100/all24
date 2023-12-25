package org.team100.lib.motion.drivetrain;

import org.team100.lib.config.Identity;

import org.team100.lib.encoder.turning.AnalogTurningEncoder.Drive;

/** Creates collections according to Identity. */
public class SwerveModuleCollectionFactory {
    // these should not have slashes
    private static final String kFrontLeft = "Front Left";
    private static final String kFrontRight = "Front Right";
    private static final String kRearLeft = "Rear Left";
    private static final String kRearRight = "Rear Right";
    private final Identity m_identity;
    private final SwerveModuleFactory m_factory;

    public SwerveModuleCollectionFactory(Identity identity, SwerveModuleFactory factory) {
        m_identity = identity;
        m_factory = factory;
    }

    public SwerveModuleCollection get() {
        switch (m_identity) {
            case COMP_BOT:
                System.out.println("************** WCP MODULES **************");
                return new SwerveModuleCollection(
                        m_factory.WCPModule(kFrontLeft, 11, 30, 0, 0.708328),
                        m_factory.WCPModule(kFrontRight, 12, 32, 1, 0.659267),
                        m_factory.WCPModule(kRearLeft, 21, 31, 2, 0.396148),
                        m_factory.WCPModule(kRearRight, 22, 33, 3, 0.109823));
            case SWERVE_TWO:
                System.out.println("************** AM CAN MODULES **************");
                return new SwerveModuleCollection(
                        m_factory.AMCANModule(kFrontLeft, 3, 36, 2, 0.354994, Drive.INVERSE),
                        m_factory.AMCANModule(kFrontRight, 12, 13, 3, 0.880423, Drive.INVERSE),
                        m_factory.AMCANModule(kRearLeft, 22, 1, 1, 0.916801, Drive.INVERSE),
                        m_factory.AMCANModule(kRearRight, 21, 0, 0, 0.806963, Drive.INVERSE));
            case SWERVE_ONE:
                System.out.println("************** AM CAN MODULES **************");
                return new SwerveModuleCollection(
                        m_factory.AMCANModule(kFrontLeft, 11, 5, 2, 0.694815, Drive.DIRECT),
                        m_factory.AMCANModule(kFrontRight, 12, 2, 0, 0.718789, Drive.DIRECT),
                        m_factory.AMCANModule(kRearLeft, 21, 3, 3, 0.365612, Drive.DIRECT),
                        m_factory.AMCANModule(kRearRight, 22, 1, 1, 0.942851, Drive.DIRECT));
            case BLANK:
                System.out.println("************** SIMULATED MODULES **************");
                return new SwerveModuleCollection(
                        m_factory.SimulatedModule(kFrontLeft),
                        m_factory.SimulatedModule(kFrontRight),
                        m_factory.SimulatedModule(kRearLeft),
                        m_factory.SimulatedModule(kRearRight));
            default:
                System.out.println("WARNING: using default module collection");
                return new SwerveModuleCollection(
                        m_factory.SimulatedModule(kFrontLeft),
                        m_factory.SimulatedModule(kFrontRight),
                        m_factory.SimulatedModule(kRearLeft),
                        m_factory.SimulatedModule(kRearRight));
        }
    }
}
