package frc.robot;

import java.util.List;

import org.dyn4j.dynamics.Body;

public abstract class Body100 extends Body {

    public static List<Class<? extends Body100>> types() {
        return List.of(Friend.class, Foe.class, Obstacle.class);
    }

    public abstract void act();
}
