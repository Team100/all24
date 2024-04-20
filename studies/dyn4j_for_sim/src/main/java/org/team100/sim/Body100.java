package org.team100.sim;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.dyn4j.collision.TypeFilter;
import org.dyn4j.dynamics.Body;

public abstract class Body100 extends Body {

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
    public static final TypeFilter NOTE = new NoteFilter();

    public static Set<String> ids = new HashSet<>();

    public Body100(String id) {
        if (ids.contains(id)) throw new IllegalArgumentException("duplicate id: " + id);
        setUserData(id);
    }

    public abstract void act();

    public String getId() {
        return (String) getUserData();
    }
}
