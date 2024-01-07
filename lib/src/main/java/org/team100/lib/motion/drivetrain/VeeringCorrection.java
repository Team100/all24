package org.team100.lib.motion.drivetrain;

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
 * The WPILib veering correction addresses the last item, about the discrete
 * control period. This is now implemented in SwerveKinodynamics, so this
 * correction here should only address the other sources of delay.
 * 
 * For more background, see CD thread:
 * https://www.chiefdelphi.com/t/field-relative-swervedrive-drift-even-with-simulated-perfect-modules/413892
 */
public class VeeringCorrection {
    /**
     * Delay in seconds.
     * 
     * In 2023 this value was 0.15. Then @calcmogul added chassis speed
     * discretization, which does the same thing, in an ideal sense. Now the delay
     * only accounts for a small amount of sensing and actuation delay, not for the
     * delay represented by the 20ms control period.
     * 
     * The value below is from preseason simulation, and it's probably too short.
     */
    private static final double kVeeringCorrection = 0.025;

    /**
     * Extrapolates the rotation based on the current angular velocity.
     * 
     * @param gyroRateRad_S current gyro rate, or the trajectory gyro rate
     * @param accelM_S magnitude of acceleration
     * @return correction amount
     */
    public static double correctionRad(double gyroRateRad_S) {
        return gyroRateRad_S * kVeeringCorrection;
    }

    private VeeringCorrection() {
        //
    }
}
