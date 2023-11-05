package org.team100.lib.motion.drivetrain.kinematics;

import org.team100.lib.motion.drivetrain.VeeringCorrection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FrameTransform {
    private final VeeringCorrection m_veering;

    public FrameTransform(VeeringCorrection veering) {
        m_veering = veering;
    }

    /**
     * Non-corrected version, identical to WPI ChassisSpeedFactory.
     */
    public FrameTransform() {
        this(new VeeringCorrection(() -> 0.0));
    }

    public Rotation2d correctAngle(Rotation2d robotAngle) {
        return m_veering.correct(robotAngle);
    }

    public ChassisSpeeds fromFieldRelativeSpeeds(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            Rotation2d robotAngle) {

        robotAngle = correctAngle(robotAngle);

        return new ChassisSpeeds(
                vxMetersPerSecond * robotAngle.getCos() + vyMetersPerSecond * robotAngle.getSin(),
                -1.0 * vxMetersPerSecond * robotAngle.getSin() + vyMetersPerSecond * robotAngle.getCos(),
                omegaRadiansPerSecond);
    }


    /**
     * Convert robot-relative speeds to field-relative speeds.
     * 
     * Does not do veering correction.
     * 
     * TODO: veering correction.
     * 
     * @param vxMetersPerSecond     robot-relative
     * @param vyMetersPerSecond     robot-relative
     * @param omegaRadiansPerSecond robot-relative
     * @return Twist2d representing field-relative motion.
     */
    public Twist2d toFieldRelativeSpeeds(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            Rotation2d robotAngle) {
        // it's just the opposite rotation
        return new Twist2d(
                vxMetersPerSecond * robotAngle.getCos() + -1.0 * vyMetersPerSecond * robotAngle.getSin(),
                vxMetersPerSecond * robotAngle.getSin() + vyMetersPerSecond * robotAngle.getCos(),
                omegaRadiansPerSecond);
    }
}
