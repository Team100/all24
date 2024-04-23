package org.team100.robot;

import java.util.NavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

import org.dyn4j.dynamics.Force;
import org.dyn4j.dynamics.Torque;
import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.Body100;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Contains the sim body. */
public class RobotSubsystem extends SubsystemBase {

    public static record Sighting(boolean friend, Translation2d position) {
    }

    // how old can sightings be and still be trusted?
    private static final double kLookbackSec = 0.1;

    /** For simulation. */
    private final RobotBody m_robotBody;

    /**
     * Some recent sightings from the camera system.
     * 
     * We can't trust that the camera knows the identity of each sighting, just the
     * position (within some tolerance) and the time (quite precisely). We can also
     * detect friend-or-foe since the bumper color tells us. Key is time in sec.
     * Note: some time jitter should be used here since one camera frame may include
     * multiple sightings.
     */
    private NavigableMap<Double, Sighting> sightings = new ConcurrentSkipListMap<>();

    public RobotSubsystem(RobotBody robotBody) {
        m_robotBody = robotBody;
    }

    public RobotBody getRobotBody() {
        return m_robotBody;
    }

    /** meters and meters per second */
    public void setState(double x, double y, double vx, double vy) {
        m_robotBody.getTransform().identity();
        m_robotBody.getTransform().translate(x, y);
        m_robotBody.setAtRest(false);
        m_robotBody.setLinearVelocity(new Vector2(vx, vy));
    }

    /** Apply force and torque. Multiple calls to this method add. */
    public void apply(double x, double y, double theta) {
        m_robotBody.applyForce(new Force(x, y));
        m_robotBody.applyTorque(new Torque(theta));
    }

    public Pose2d getPose() {
        return m_robotBody.getPose();
    }

    public FieldRelativeVelocity getVelocity() {
        return m_robotBody.getVelocity();
    }

    @Override
    public void periodic() {
        lookAround();
    }

    /**
     * In reality, each robot would get camera updates asynchronously (and
     * potentially out-of-order) but to keep it simple (and because dyn4j is not
     * thread safe), it's here.
     */
    private void lookAround() {
        for (Body100 body : m_robotBody.getWorld().getBodies()) {
            if (body == m_robotBody) {
                // skip ourselves
                continue;
            }
            if (!(body instanceof RobotBody)) {
                // look only at robots
                continue;
            }
            RobotBody robotBody = (RobotBody) body;
            Vector2 targetPosition = robotBody.getWorldCenter();
            boolean friend = robotBody.friend(m_robotBody);
            addSighting(friend, targetPosition);
        }
    }

    /** Add a sighting with the current timestamp. */
    private void addSighting(boolean friend, Vector2 fieldRelativePosition) {
        double now = Timer.getFPGATimestamp();
        Sighting sighting = new Sighting(
                friend,
                new Translation2d(fieldRelativePosition.x, fieldRelativePosition.y));
        sightings.put(now, sighting);
    }

    public NavigableMap<Double, Sighting> recentSightings() {
        double now = Timer.getFPGATimestamp();
        // first trim the sightings to remove stale ones
        sightings.keySet().removeAll(sightings.headMap(now - kLookbackSec).keySet());
        return sightings.descendingMap();
    }

    public Vector2 shootingPosition() {
        return m_robotBody.shootingPosition();
    }

    public Vector2 ampPosition() {
        return m_robotBody.ampPosition();
    }

    public Vector2 sourcePosition() {
        return m_robotBody.sourcePosition();
    }
}
