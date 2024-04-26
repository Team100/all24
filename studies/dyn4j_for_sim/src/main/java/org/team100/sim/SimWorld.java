package org.team100.sim;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.dyn4j.dynamics.joint.Joint;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.robot.FieldMap;
import org.team100.robot.Scorekeeper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * In this world, the player and friends are blue, the foes are red.
 */
public class SimWorld {
    private static final double boundaryThickness = 1;
    private static final double fieldX = 16.541;
    private static final double fieldY = 8.211;
    // this is the actual amp length.
    // private static final double ampLength = 2.418;
    // amp length only goes to (1mm short of) the pocket
    private static final double ampLength = 1.535;
    // this is just beyond the pocket
    private static final double topWallLimit = 2.146;
    private static final double ampHeight = 1.207;
    private static final double allianceWallHeightM = 1.983;
    private static final String kField = "field";
    private static final Telemetry t = Telemetry.get();

    private final World<Body100> world;
    // this is a copy of the obstacle translations since we use this all the time.
    private final List<Translation2d> obstacles;

    public SimWorld() {
        world = new World<>();
        world.setGravity(PhysicsWorld.ZERO_GRAVITY);
        world.setValueMixer(new ValueMixer100());
        world.setBounds(new Bounds100());

        setUpWalls();
        setUpStages();
        setUpNotes();
        setUpSensors();

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

    public void addJoint(Joint<Body100> joint) {
        world.addJoint(joint);
    }

    public boolean removeJoint(Joint<Body100> joint) {
        return world.removeJoint(joint);
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

    /** Speakers and amp pockets are sensors. */
    private void setUpSensors() {
        Speaker blueSpeaker = new Speaker("blue speaker",
                Geometry.createPolygon(
                        new Vector2(0, 5.018),
                        new Vector2(0, 6.075),
                        new Vector2(-3, 6.075),
                        new Vector2(-3, 5.018)));
        world.addBody(blueSpeaker);
        Speaker redSpeaker = new Speaker("red speaker",
                Geometry.createPolygon(
                        new Vector2(19.541, 5.018),
                        new Vector2(19.541, 6.075),
                        new Vector2(16.541, 6.075),
                        new Vector2(16.541, 5.018)));
        world.addBody(redSpeaker);
        AmpPocket blueAmp = new AmpPocket("blue amp pocket",
                Geometry.createPolygon(
                        new Vector2(1.536, fieldY),
                        new Vector2(2.145, fieldY),
                        new Vector2(2.145, fieldY + boundaryThickness),
                        new Vector2(1.536, fieldY + boundaryThickness)));
        world.addBody(blueAmp);
        AmpPocket redAmp = new AmpPocket("red amp pocket",
                Geometry.createPolygon(
                        new Vector2(14.394, fieldY),
                        new Vector2(15.003, fieldY),
                        new Vector2(15.003, fieldY + boundaryThickness),
                        new Vector2(14.394, fieldY + boundaryThickness)));
        world.addBody(redAmp);
        Scorekeeper scorekeeper = new Scorekeeper(blueSpeaker, redSpeaker, blueAmp, redAmp);
        world.addCollisionListener(scorekeeper);
        world.addBoundsListener(scorekeeper);
        world.addStepListener(scorekeeper);
    }

    /**
     * this uses simgui coordinates for blue
     */
    private void setUpWalls() {
        world.addBody(
                new Wall("blue source",
                        Geometry.createTriangle(
                                new Vector2(0, 0),
                                new Vector2(1.84, 0),
                                new Vector2(0, 1.1)),
                        1.695));
        world.addBody(
                new Wall("red source",
                        Geometry.createTriangle(
                                new Vector2(16.541, 0),
                                new Vector2(16.541, 1.1),
                                new Vector2(14.7, 0)),
                        1.695));
        // TODO: something about the subwoofer vertical shape
        // extends way past the boundary so that notes won't get stuck between the wall
        // and subwoofer.
        world.addBody(
                new Wall("blue subwoofer",
                        Geometry.createPolygon(
                                new Vector2(-3, 4.498),
                                new Vector2(0, 4.498),
                                new Vector2(0.914, 5.019),
                                new Vector2(0.914, 6.062),
                                new Vector2(0, 6.597),
                                new Vector2(-3, 6.597)),
                        0.213));
        world.addBody(
                new Wall("red subwoofer",
                        Geometry.createPolygon(
                                new Vector2(16.541, 4.498),
                                new Vector2(19.541, 4.498),
                                new Vector2(19.541, 6.597),
                                new Vector2(16.541, 6.597),
                                new Vector2(15.6, 6.062),
                                new Vector2(15.6, 5.019)),
                        0.213));
        world.addBody(
                new Wall("blue wall",
                        Geometry.createPolygon(
                                new Vector2(0, 0),
                                new Vector2(0, fieldY),
                                new Vector2(-boundaryThickness, fieldY),
                                new Vector2(-boundaryThickness, 0)),
                        allianceWallHeightM));
        world.addBody(
                new Wall("red wall",
                        Geometry.createPolygon(
                                new Vector2(fieldX, fieldY),
                                new Vector2(fieldX, 0),
                                new Vector2(fieldX + boundaryThickness, 0),
                                new Vector2(fieldX + boundaryThickness, fieldY)),
                        allianceWallHeightM));
        world.addBody(
                new Wall("top wall",
                        Geometry.createPolygon(
                                new Vector2(topWallLimit, fieldY),
                                new Vector2(fieldX - topWallLimit, fieldY),
                                new Vector2(fieldX - topWallLimit, fieldY + boundaryThickness),
                                new Vector2(topWallLimit, fieldY + boundaryThickness)),
                        0.508));
        world.addBody(
                new Wall("bottom wall",
                        Geometry.createPolygon(
                                new Vector2(fieldX, 0),
                                new Vector2(0, 0),
                                new Vector2(0, -boundaryThickness),
                                new Vector2(fieldX, -boundaryThickness)),
                        0.508));
        // this is a bit simplified: the "amp" wall extends from the corner to the amp
        // scoring pocket.
        world.addBody(
                new Wall("blue amp",
                        Geometry.createPolygon(
                                new Vector2(0, fieldY),
                                new Vector2(ampLength, fieldY),
                                new Vector2(ampLength, fieldY + boundaryThickness),
                                new Vector2(0, fieldY + boundaryThickness)),
                        ampHeight));
        world.addBody(
                new Wall("red amp",
                        Geometry.createPolygon(
                                new Vector2(fieldX - ampLength, fieldY),
                                new Vector2(fieldX, fieldY),
                                new Vector2(fieldX, fieldY + boundaryThickness),
                                new Vector2(fieldX - ampLength, fieldY + boundaryThickness)),
                        ampHeight));

        // TODO: add amp wall
        // TODO: add scoring sensors
    }

    private void setUpStages() {
        // these are from the onshape cad,
        // adjusted a tiny bit to line up with the background image.
        for (Map.Entry<String, Pose2d> post : FieldMap.stagePosts.entrySet()) {
            String name = post.getKey();
            Pose2d pose = post.getValue();
            addPost(name, pose.getX(), pose.getY(), pose.getRotation().getRadians());
        }
        // TODO: add stage body
    }

    private void addPost(String id, double x, double y, double rad) {
        Body100 post = new Obstacle(
                id,
                Geometry.createSquare(0.3),
                1.878);
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

    public void addNote(double x, double y) {
        Note note = new Note();
        note.translate(x, y);
        world.addBody(note);
        world.addStepListener(note);
    }

}
