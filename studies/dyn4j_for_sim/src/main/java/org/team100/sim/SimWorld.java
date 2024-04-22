package org.team100.sim;

import java.util.ArrayList;
import java.util.List;

import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * In this world, the player and friends are blue, the foes are red.
 */
public class SimWorld {
    private static final String kField = "field";
    private static final Telemetry t = Telemetry.get();

    private final World<Body100> world;
    // this is a copy of the obstacle translations since we use this all the time.
    private final List<Translation2d> obstacles;

    public SimWorld() {
        world = new World<>();
        world.setGravity(PhysicsWorld.ZERO_GRAVITY);

        setUpWalls();
        setUpStages();
        setUpNotes();

        // cache the obstacle locations since we use them all the time.
        obstacles = new ArrayList<>();
        for (Body100 body : world.getBodies()) {
            if (body instanceof Obstacle) {
                Vector2 translation = body.getTransform().getTranslation();
                obstacles.add(new Translation2d(translation.x, translation.y));
            }
        }

        // need the .type for rendering the field2d in sim.
        t.log(Level.INFO, "field", ".type", "Field2d");
    }

    public void addBody(RobotBody body) {
        world.addBody(body);
    }

    public void update() {
        // update the dyn4j sim
        world.update(0.02);
    }

    public List<Body100> getBodies() {
        return world.getBodies();
    }
    
    public List<Translation2d> getObstacles() {
        return obstacles;
    }

    /** Show the bodies on the field2d widget */
    public void render() {
        // each type is its own array for the field2d widget :-(
        for (Class<?> type : Body100.types()) {
            List<Double> poses = new ArrayList<>();
            for (int i = 0; i < world.getBodyCount(); ++i) {
                Body100 body = world.getBody(i);
                if (body.getClass() != type)
                    continue;
                Vector2 positionM = body.getWorldCenter();
                double angleDeg = body.getTransform().getRotation().toDegrees();
                poses.add(positionM.x);
                poses.add(positionM.y);
                poses.add(angleDeg);
            }
            t.log(Level.DEBUG, kField, type.getSimpleName(),
                    poses.toArray(new Double[0]));
        }
    }

    /*
     * make the NPC's do something interesting.
     * 
     * what are the units here? newtons?
     */
    public void behavior() {
        for (Body100 body : world.getBodies()) {
            body.act();
        }
    }

    private void setUpWalls() {
        // this uses simgui coordinates for blue
        final double boundaryThickness = 1;
        final double fieldX = 16.541;
        final double fieldY = 8.211;
        world.addBody(new Wall("blue source",
                Geometry.createTriangle(
                        new Vector2(0, 0),
                        new Vector2(1.84, 0),
                        new Vector2(0, 1.1))));
        world.addBody(new Wall("red source",
                Geometry.createTriangle(
                        new Vector2(16.541, 0),
                        new Vector2(16.541, 1.1),
                        new Vector2(14.7, 0))));
        world.addBody(new Wall("blue subwoofer",
                Geometry.createPolygon(
                        new Vector2(0, 4.498),
                        new Vector2(0.914, 5.019),
                        new Vector2(0.914, 6.062),
                        new Vector2(0, 6.597))));
        world.addBody(new Wall("red subwoofer",
                Geometry.createPolygon(
                        new Vector2(16.541, 4.498),
                        new Vector2(16.541, 6.597),
                        new Vector2(15.6, 6.062),
                        new Vector2(15.6, 5.019))));
        world.addBody(new Wall("blue wall",
                Geometry.createPolygon(
                        new Vector2(0, 0),
                        new Vector2(0, fieldY),
                        new Vector2(-boundaryThickness, fieldY),
                        new Vector2(-boundaryThickness, 0))));
        world.addBody(new Wall("red wall",
                Geometry.createPolygon(
                        new Vector2(fieldX, fieldY),
                        new Vector2(fieldX, 0),
                        new Vector2(fieldX + boundaryThickness, 0),
                        new Vector2(fieldX + boundaryThickness, fieldY))));
        world.addBody(new Wall("top wall",
                Geometry.createPolygon(
                        new Vector2(0, fieldY),
                        new Vector2(fieldX, fieldY),
                        new Vector2(fieldX, fieldY + boundaryThickness),
                        new Vector2(0, fieldY + boundaryThickness))));
        world.addBody(new Wall("bottom wall",
                Geometry.createPolygon(
                        new Vector2(fieldX, 0),
                        new Vector2(0, 0),
                        new Vector2(0, -boundaryThickness),
                        new Vector2(fieldX, -boundaryThickness))));
    }

    private void setUpStages() {
        // these are from the onshape cad,
        // adjusted a tiny bit to line up with the background image.
        addPost("east post", 3.38, 4.10, 0);
        addPost("southeast post", 5.60, 2.80, -1);
        addPost("northeast post", 5.60, 5.38, 1);
        addPost("southwest post", 10.95, 2.80, 1);
        addPost("northwest post", 10.95, 5.38, -1);
        addPost("west post", 13.16, 4.10, 0);
    }

    private void addPost(String id, double x, double y, double rad) {
        Body100 post = new Obstacle(id, Geometry.createSquare(0.3));
        post.rotate(rad);
        post.translate(x, y);
        world.addBody(post);
    }

    private void setUpNotes() {
        // these are just made to match the background image
        addNote(2.890, 4.10);
        addNote(2.890, 5.56);
        addNote(2.890, 7.01);

        addNote(8.275, 0.75);
        addNote(8.275, 2.43);
        addNote(8.275, 4.10);
        addNote(8.275, 5.79);
        addNote(8.275, 7.47);

        addNote(13.657, 4.10);
        addNote(13.657, 5.56);
        addNote(13.657, 7.01);
    }

    private void addNote(double x, double y) {
        Body100 post = new Note(Geometry.createCircle(0.175));
        post.translate(x, y);
        world.addBody(post);
    }

}
