package org.team100.sim;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract class RobotBody extends Body100 {
    protected static final double kRobotSize = 0.75;

    private final SimWorld m_world;

    protected RobotBody(String id, SimWorld world) {
        super(id);
        m_world = world;

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

    public abstract boolean friend(RobotBody body);

    public abstract Vector2 ampPosition();

    public abstract Vector2 shootingPosition();

    public abstract double shootingAngle();

    public abstract Vector2 sourcePosition();

    public SimWorld getWorld() {
        return m_world;
    }

    public Pose2d getPose() {
        Transform simTransform = transform;
        Vector2 translation = simTransform.getTranslation();
        double angle = simTransform.getRotationAngle();
        return new Pose2d(
                translation.x,
                translation.y,
                new Rotation2d(angle));
    }

    public FieldRelativeVelocity getVelocity() {
        return new FieldRelativeVelocity(
                linearVelocity.x,
                linearVelocity.y,
                angularVelocity);
    }
}
