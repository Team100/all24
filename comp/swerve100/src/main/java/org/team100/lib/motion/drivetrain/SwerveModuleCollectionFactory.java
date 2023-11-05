package org.team100.lib.motion.drivetrain;

import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiments;

/** Creates collections according to Identity. */
public class SwerveModuleCollectionFactory {
    private final Identity identity;
    private final SwerveModuleFactory m_moduleFactory;

    public SwerveModuleCollectionFactory(
            Experiments experiments,
            Identity identity,
            double currentLimit) {
        this.identity = identity;
        m_moduleFactory = new SwerveModuleFactory(experiments, currentLimit);
    }

    public SwerveModuleCollectionInterface get() {
        switch (identity) {
            case COMP_BOT:
                return new SwerveModuleCollection(
                        m_moduleFactory.WCPModule(
                                "Front Left",
                                11, // drive CAN
                                30, // turn CAN
                                0, // turn encoder
                                0.708328), // turn offset
                        m_moduleFactory.WCPModule(
                                "Front Right",
                                12, // drive CAN
                                32, // turn CAN
                                1, // turn encoder
                                0.659267), // turn offset
                        m_moduleFactory.WCPModule(
                                "Rear Left",
                                21, // drive CAN
                                31, // turn CAN
                                2, // turn encoder
                                0.396148), // turn offset
                        m_moduleFactory.WCPModule(
                                "Rear Right",
                                22, // drive CAN
                                33, // turn CAN
                                3, // turn encoder
                                0.109823)); // turn offset
            case SWERVE_TWO:
                return new SwerveModuleCollection(
                    m_moduleFactory.Swerve2CAN(
                        "Front Left",
                        3, // drive CAN
                        36, // turn PWM
                        2, // turn encoder
                        0.354994), // turn offset
                    m_moduleFactory.Swerve2CAN(
                        "Front Right",
                        12, // drive CAN
                        13, // turn PWM
                        3, // turn encoder
                        0.880423), // turn offset
                   m_moduleFactory.Swerve2CAN(
                        "Rear Left",
                        22, // drive CAN
                        1 ,// turn PWM
                        1, // turn encoder
                        0.916801), // turn offset
                  m_moduleFactory.Swerve2CAN(
                        "Rear Right",
                        21, // drive CAN
                        0, // turn PWM
                        0, // turn encoder
                        0.806963)); // turn offset
            case SWERVE_ONE:
                return new SwerveModuleCollection(
                        m_moduleFactory.Swerve1CAN(
                                "Front Left",
                                11, // drive CAN
                                5, // turn PWM0
                                2, // turn encoder
                                0.694815), // turn offset
                        m_moduleFactory.Swerve1CAN(
                                "Front Right",
                                12, // drive CAN
                                2, // turn PWM
                                0, // turn encoder
                                0.718789), // turn offset
                        m_moduleFactory.Swerve1CAN(
                                "Rear Left",
                                21, // drive CAN
                                3, // turn PWM
                                3, // turn encoder
                                0.365612), // turn offset
                        m_moduleFactory.Swerve1CAN(
                                "Rear Right",
                                22, // drive CAN
                                1, // turn PWM
                                1, // turn encoder
                                0.942851)); // turn offset
            default:
         return new SwerveModuleCollection.Noop();
            // previously this would throw.
            // throw new IllegalStateException("Identity is not swerve: " +
            // Identity.get().name());
        }
    }

}
