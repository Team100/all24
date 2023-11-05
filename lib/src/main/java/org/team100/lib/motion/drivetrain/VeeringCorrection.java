package org.team100.lib.motion.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Corrects the tendency of the swerve drive to veer in the direction of
 * rotation, which is caused by delay in the sense->actuate loop.
 * 
 * Sources of delay include
 * 
 * * velocity window size
 * * velocity low-pass filtering
 * * steering controller delay
 * * robot period (actuation for 20 ms in the future, not just right now)
 * 
 * This issue is discussed in this CD thread:
 * https://www.chiefdelphi.com/t/field-relative-swervedrive-drift-even-with-simulated-perfect-modules/413892
 * 
 * TODO: WPILib supports veering correction now; try their version of it.
 */
public class VeeringCorrection {
    public static class Config {
        /**
         * Delay in seconds.
         */
        public double kVeeringCorrection = 0.15;
    }

    private final Config m_config = new Config();

    private final DoubleSupplier m_gyroRateRadSNWU;

    /**
     * Be sure to supply NWU, counterclockwise-positive, rates here. The default
     * AHRS rate is the other way, don't use that without inverting it. Use the
     * Heading class.
     * 
     * @param gyroRateRadSNWU rotation rate counterclockwise positive rad/s
     */
    public VeeringCorrection(DoubleSupplier gyroRateRadSNWU) {
        m_gyroRateRadSNWU = gyroRateRadSNWU;
    }

    /**
     * Extrapolates the rotation based on the current angular velocity.
     * 
     * @param in past rotation
     * @return future rotation
     */
    public Rotation2d correct(Rotation2d in) {
        return in.plus(new Rotation2d(m_gyroRateRadSNWU.getAsDouble() * m_config.kVeeringCorrection));
    }

}
