package frc.robot;

import java.util.List;

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

    public abstract void act();
}
