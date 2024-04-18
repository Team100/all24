package frc.robot;

import java.util.List;

import org.dyn4j.dynamics.Body;

public abstract class Body100 extends Body {

    /** This is the list of types that will be rendered. */
    public static List<Class<? extends Body100>> types() {
        return List.of(
                Player.class,
                Friend.class,
                Foe.class,
                Obstacle.class);
    }

    public abstract void act();
}
