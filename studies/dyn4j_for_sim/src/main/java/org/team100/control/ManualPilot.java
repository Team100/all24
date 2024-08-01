package org.team100.control;

import java.lang.reflect.Field;
import java.util.Map;

import org.team100.field.FieldMap;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.ForceViz;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

/** A human driver/operator */
public class ManualPilot implements Pilot {
    private final XboxController m_controller = new XboxController(0);
    private final DriveSubsystem m_drive;
    private static final double kSubwooferRepulsion = 5;
    private static final double kObstacleRepulsion = 10;

    boolean m_debug = true;

    public ManualPilot(DriveSubsystem drive) {
        m_drive = drive;
    }

    @Override
    public FieldRelativeVelocity driveVelocity() {

        FieldRelativeVelocity v = new FieldRelativeVelocity(
                -m_controller.getRightY() * Kinodynamics.kMaxVelocity, // axis 5
                -m_controller.getRightX() * Kinodynamics.kMaxVelocity, // axis 4
                -m_controller.getLeftX() * Kinodynamics.kMaxOmega); // axis 0

        return apply(v);

    }

    public FieldRelativeVelocity apply(FieldRelativeVelocity desired) {
        Pose2d myPosition = m_drive.getPose();
        final double maxDistance = 1.5;
        FieldRelativeVelocity v = desired;
        for (Pose2d pose : FieldMap.stagePosts.values()) {
            Translation2d target = pose.getTranslation();
            Translation2d robotRelativeToTarget = myPosition.getTranslation().minus(target);
            double norm = robotRelativeToTarget.getNorm();
            if (norm < maxDistance) {
                // unit vector in the direction of the force
                Translation2d normalized = robotRelativeToTarget.div(norm);
                // scale the force so that it's zero at the maximum distance, i.e. C0 smooth.
                // the minimum distance is something like 0.75, so
                // the maximum force is (1.3-0.3) = 0.6 * k
                double scale = kObstacleRepulsion * (1 / norm - 1 / maxDistance);
                Translation2d force = normalized.times(scale);
                if (m_debug)
                    System.out.printf(" obstacleRepulsion (%5.2f, %5.2f)", force.getX(), force.getY());
                FieldRelativeVelocity repel = new FieldRelativeVelocity(force.getX(), force.getY(), 0);
                v = v.plus(repel);
            }
        }
        return v;
    }

    @Override
    public boolean intake() {
        return m_controller.getRawButton(1);
    }

    @Override
    public boolean outtake() {
        return m_controller.getRawButton(2);
    }

    @Override
    public boolean shoot() {
        return m_controller.getRawButton(3);
    }

    @Override
    public boolean lob() {
        return m_controller.getRawButton(4);
    }

    @Override
    public boolean amp() {
        return m_controller.getRawButton(5);
    }

    @Override
    public boolean rotateToShoot() {
        return m_controller.getRawButton(6);
    }

    @Override
    public boolean scoreSpeaker() {
        return m_controller.getRawButton(7);
    }

    @Override
    public boolean scoreAmp() {
        return m_controller.getRawButton(8);
    }

    @Override
    public boolean driveToSource() {
        return m_controller.getRawButton(9);
    }

    @Override
    public boolean pass() {
        return m_controller.getRawButton(10);
    }

    @Override
    public boolean shootCommand() {
        return m_controller.getRawButton(11);
    }
}
