package org.team100.sim;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

    protected static final double kRobotSize = 0.75;
    protected final SimWorld m_world;
    protected Goal m_goal;

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

    public abstract boolean friend(RobotBody body);

    abstract Vector2 ampPosition();

    abstract Vector2 shootingPosition();

    public abstract double shootingAngle();

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
            applyTorque(angleError * 50);
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
