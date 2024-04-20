package org.team100.sim;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.World;

public abstract class RobotBody extends Body100 {
    private static final int kSteer = 500;
    protected final World<Body100> m_world;

    protected RobotBody(World<Body100> world) {
        m_world = world;

        // about 30 inches including bumpers == 24 inch frame
        // 100 kg/m2 implies about 120 lbs
        // 0.5 friction is a total guess
        // 0.1 restitution: bumpers are not springy
        BodyFixture fixture = addFixture(
                Geometry.createSquare(0.75),
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

    protected void avoidObstacles() {
        Vector2 position = getWorldCenter();
        Vector2 velocity = getLinearVelocity();
        for (Body100 body : m_world.getBodies()) {
            if (body == this)
                continue;
            Vector2 targetPosition = body.getWorldCenter();
            double distance = position.distance(targetPosition);
            if (distance > 4) // ignore far-away obstacles
                continue;
            if (body instanceof Obstacle) {
                Vector2 steer = Heuristics.steerToAvoid(
                        position, velocity, targetPosition, 1);
                if (steer.getMagnitude() < 1e-3)
                    continue;
                Vector2 force = steer.product(kSteer);
                applyForce(force);
            }
        }
    }

    protected void avoidRobots() {
        // TODO: estimate robot velocity
        Vector2 position = getWorldCenter();
        Vector2 velocity = getLinearVelocity();
        for (Body100 body : m_world.getBodies()) {
            if (body == this)
                continue;
            Vector2 targetPosition = body.getWorldCenter();
            double distance = position.distance(targetPosition);
            if (distance > 4) // ignore far-away obstacles
                continue;
            if (body instanceof RobotBody) {
                Vector2 steer = Heuristics.steerToAvoid(
                        position, velocity, targetPosition, 1.25);
                if (steer.getMagnitude() < 1e-3)
                    continue;
                Vector2 force = steer.product(kSteer);
                applyForce(force);
            }
        }
    }

    protected void avoidEdges() {
        Vector2 position = getWorldCenter();

        // avoid the edges of the field
        if (position.x < 1)
            applyForce(new Force(100, 0));
        if (position.x > 15)
            applyForce(new Force(-100, 0));
        if (position.y < 1)
            applyForce(new Force(0, 100));
        if (position.y > 7)
            applyForce(new Force(0, -100));
    }
}
