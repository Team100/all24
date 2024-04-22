package org.team100.sim;

import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public abstract class RobotBody extends Body100 {

    /** Robots pursue one of these goals at a time. */
    public enum Goal {
        /** Drive to the source corner and pick up a note. */
        PICK,
        /** Drive to the amp and place. */
        SCORE_AMP,
        /** Drive to a shooting spot and shoot. */
        SCORE_SPEAKER,
        /** Drive to a passing spot and lob. */
        PASS,
        /** Drive towards the closest note and take it. */
        STEAL,
        /** Stay between the foes and their source */
        DEFEND_SOURCE,
        /** Apply no force */
        NOTHING
    }

    private static record Sighting(boolean friend, Vector2 position) {
    }

    private static final int kSteer = 500;
    protected static final double kRobotSize = 0.75;
    protected final SimWorld m_world;
    protected Goal m_goal;

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

    protected RobotBody(String id, SimWorld world, Goal initialGoal) {
        super(id);
        m_world = world;
        m_goal = initialGoal;

        // about 30 inches including bumpers == 24 inch frame
        // 100 kg/m2 implies about 120 lbs
        // 0.5 friction is a total guess
        // 0.1 restitution: bumpers are not springy
        BodyFixture fixture = addFixture(
                Geometry.createSquare(kRobotSize),
                100,
                0.5,
                0.1);
        // this means the springiness doesn't change with velocity
        fixture.setRestitutionVelocity(0.0);
        fixture.setFilter(ROBOT);
        setMass(MassType.NORMAL);
        // fiddled with damping until it seemed "right"
        setAngularDamping(5);
        setLinearDamping(0.75);
    }

    public SimWorld getWorld() {
        return m_world;
    }

    public void setGoal(Goal goal) {
        m_goal = goal;
    }

    abstract boolean friend(RobotBody body);

    abstract Vector2 ampPosition();

    abstract Vector2 shootingPosition();

    abstract double shootingAngle();

    // how old can sightings be and still be trusted?
    private static final double kLookbackSec = 0.1;
    // targets appearing to move faster than this are probably false associations.
    private static final double kMaxTargetVelocity = 4;

    /**
     * Track the bearing to each robot.
     */
    protected void avoidRobots() {
        double now = Timer.getFPGATimestamp();
        // first trim the sightings to remove stale ones
        sightings.keySet().removeAll(sightings.headMap(now - kLookbackSec).keySet());

        Vector2 position = getWorldCenter();
        Vector2 velocity = getLinearVelocity();
        for (Body100 body : m_world.getBodies()) {

            if (body == this)
                continue;
            if (!(body instanceof RobotBody))
                continue;
            RobotBody robotBody = (RobotBody) body;

            // assume the camera can give us the relative position of the body
            Vector2 targetPosition = robotBody.getWorldCenter();
            boolean friend = robotBody.friend(this);

            // have we seen something nearby lately?
            for (Entry<Double, Sighting> entry : sightings.descendingMap().entrySet()) {
                if (entry.getValue().friend != friend)
                    continue;
                // same type, so maybe the same robot?
                Vector2 targetVelocity = targetPosition.difference(
                        entry.getValue().position).quotient(now - entry.getKey());
                if (targetVelocity.getMagnitude() > kMaxTargetVelocity)
                    continue;
                // reasonable velocity
                // TODO: do something with the target velocity.
            }

            Sighting sighting = new Sighting(friend, targetPosition);
            sightings.put(now, sighting);

            double distance = position.distance(targetPosition);
            if (distance > 4) // don't react to far-away obstacles
                continue;

            // treat the target as a fixed obstacle.
            Vector2 steer = Heuristics.steerToAvoid(
                    position, velocity, targetPosition, 1.25);
            if (steer.getMagnitude() < 1e-3)
                continue;
            Vector2 force = steer.product(kSteer);
            applyForce(force);
        }
    }

    public Pose2d getPose() {
        Transform simTransform = transform;
        Vector2 translation = simTransform.getTranslation();
        double angle = simTransform.getRotationAngle();
        return new Pose2d(translation.x, translation.y, new Rotation2d(angle));
    }

    public FieldRelativeVelocity getVelocity() {
        return new FieldRelativeVelocity(linearVelocity.x, linearVelocity.y, angularVelocity);
    }

    Goal m_previousScoring = Goal.NOTHING;

    /**
     * this is the "score" behavior which just drives to the amp.
     * 
     * TODO: add the amp mechanics
     * TODO: add shooting
     */
    protected void driveToAmp() {
        Vector2 position = transform.getTranslation();
        Vector2 positionError = position.to(ampPosition());
        double angle = transform.getRotationAngle();
        // TODO: remove this magic angle
        double angleError = MathUtil.angleModulus(Math.PI / 2 - angle);
        // amp tolerance is quite low
        if (Math.abs(positionError.x) < 0.1
                && Math.abs(positionError.y) < 0.05
                && Math.abs(angleError) < 0.05) {
            // successful score
            nextGoal();
        } else {
            // keep trying
            applyForce(positionError.setMagnitude(200));
            applyTorque(angleError * 10);
        }
    }

    /** TODO: replace with a more general driving plan */
    protected void driveToSpeaker() {
        Vector2 position = transform.getTranslation();
        Vector2 positionError = position.to(shootingPosition());
        double angle = transform.getRotationAngle();
        // TODO: remove this magic angle
        double angleError = MathUtil.angleModulus(shootingAngle() - angle);
        // speaker position tolerance is loose but angle is not
        if (Math.abs(positionError.x) < 0.5
                && Math.abs(positionError.y) < 0.5
                && Math.abs(angleError) < 0.05) {
            // successful score
            nextGoal();
        } else {
            // keep trying
            applyForce(positionError.setMagnitude(200));
            applyTorque(angleError * 10);
        }
    }

    /**
     * After each goal is achieved, this goes to the next goal.
     * TODO: replace this whole thing with commands or triggers or whatever.
     */
    protected void nextGoal() {
        switch (m_goal) {
            case PICK:
                if (m_previousScoring == Goal.SCORE_AMP) {
                    m_goal = Goal.SCORE_SPEAKER;
                    m_previousScoring = m_goal;
                } else if (m_previousScoring == Goal.SCORE_SPEAKER) {
                    m_goal = Goal.SCORE_AMP;
                    m_previousScoring = m_goal;
                } else {
                    // default to the amp first
                    m_goal = Goal.SCORE_AMP;
                    m_previousScoring = m_goal;
                }
                break;
            case SCORE_AMP:
            case SCORE_SPEAKER:
                m_goal = Goal.PICK;
                break;
            default:
                m_goal = Goal.PICK;
        }
    }
}
