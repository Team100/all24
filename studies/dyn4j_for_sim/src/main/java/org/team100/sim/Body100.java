package org.team100.sim;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.dyn4j.collision.TypeFilter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.TimeStep;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.listener.StepListener;

public abstract class Body100 extends Body implements StepListener<Body100> {

    /**
     * This is the list of types that will be rendered.
     * 
     * We don't try to render the {@link Wall} type because simgui can't do it
     * correctly.
     */
    public static List<Class<? extends Body100>> types() {
        return List.of(
                Friend.class,
                Foe.class,
                Note.class,
                Obstacle.class,
                Player.class);
    }

    public static final TypeFilter FIXED = new FixedFilter();
    public static final TypeFilter ROBOT = new RobotFilter();

    private static final Set<String> ids = new HashSet<>();

    protected Body100(String id) {
        if (ids.contains(id))
            throw new IllegalArgumentException("duplicate id: " + id);
        setUserData(id);
    }

    @Override
    public void begin(TimeStep step, PhysicsWorld<Body100, ?> world) {
        //
    }

    @Override
    public void updatePerformed(TimeStep step, PhysicsWorld<Body100, ?> world) {
        //
    }

    @Override
    public void postSolve(TimeStep step, PhysicsWorld<Body100, ?> world) {
        //
    }

    @Override
    public void end(TimeStep step, PhysicsWorld<Body100, ?> world) {
        //
    }
}
