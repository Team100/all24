package org.team100.robot;

import org.team100.sim.Note;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The shooter is used for shooting, lobbing, and amping. */
public class ShooterSubsystem extends SubsystemBase {

    /**
     * Shooter impulse is measured in newton-seconds.
     * For a 0.235 kg note at 20 m/s the impulse is about 4.7 Ns.
     */
    private static final double kShootImpulseNs = 4.7;

    /**
     * Shooter angle is actually variable
     * TODO: implement shooter map
     */
    // private static final double kShootElevationRad = -0.5;

    /**
     * Lob is slower, enough to get about 3.5m high and travel about 6m
     */
    private static final double kLobImpulseNs = 3.0;

    /**
     * Lob uses a pretty high angle, this is a guess.
     */
    private static final double kLobElevationRad = -1.0;

    /**
     * Amp shot is gentle
     */
    private static final double kAmpImpulseNs = 1.2;

    /**
     * Amp elevation is very high
     */
    private static final double kAmpElevationRad = -1.48;

    private final RobotAssembly m_assembly;

    private final RobotBody m_robotBody;
    /**
     * key == range
     * value == pitch
     * minimum range == 1.29
     */
    InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
    private final Translation2d m_speakerPosition;

    public ShooterSubsystem(RobotAssembly assembly, RobotBody robotBody, Translation2d speakerPosition) {
        m_assembly = assembly;

        m_robotBody = robotBody;
        m_speakerPosition = speakerPosition;
        // I experimented and then smoothed the curve by eye.
        shooterMap.put(0.0, -1.571); // pi/2
        shooterMap.put(0.5, -1.400);
        shooterMap.put(1.0, -1.240); // 1.29 is min feasible range
        shooterMap.put(1.5, -1.070);
        shooterMap.put(2.0, -0.930);
        shooterMap.put(2.5, -0.810);
        shooterMap.put(3.0, -0.710);
        shooterMap.put(3.5, -0.650);
        shooterMap.put(4.0, -0.590);
        shooterMap.put(4.5, -0.540);
        shooterMap.put(5.0, -0.500);
        shooterMap.put(5.5, -0.460);
        shooterMap.put(6.0, -0.435);
        shooterMap.put(6.5, -0.415);
        shooterMap.put(7.0, -0.400); // beyond max feasible range
    }

    /** Shooting is full speed, 20 m/s */
    public void shoot() {
        Note n = m_assembly.m_indexerShooterHandoff;
        if (n == null)
            return;

        double range = m_robotBody.getPose().getTranslation().getDistance(m_speakerPosition);
        double elevationRad = shooterMap.get(range);
        System.out.printf("shoot range %5.3f elevation %5.3f\n",
                range, elevationRad);
        shooter(kShootImpulseNs, elevationRad, 0, n);
    }

    /** Lob uses a lower exit velocity. */
    public void lob(Note n) {
        shooter(kLobImpulseNs, kLobElevationRad, 0, n);
    }

    /**
     * Amp uses very low velocity, opposite rotation.
     * 
     * TODO: remove the yaw, make the elevation > pi/2.
     */
    public void amp(Note n) {
        shooter(kAmpImpulseNs, kAmpElevationRad, Math.PI, n);
    }

    /** TODO: remove yaw offset */
    private void shooter(double impulseNs, double elevationRad, double yawOffsetRad, Note n) {
        double yawRad = m_robotBody.getPose().getRotation().getRadians();
        yawRad = MathUtil.angleModulus(yawRad + yawOffsetRad);
        Translation3d t3 = new Translation3d(
                impulseNs,
                new Rotation3d(0, elevationRad, yawRad));
        n.applyImpulse(t3);
    }

}
