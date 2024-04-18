package frc.robot;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.World;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private static final String kField = "field";
    private static final String kBall = "ball";
    private static final Telemetry t = Telemetry.get();

    protected final World<Body> world;
    private Body ball;

    public Robot() {
        world = new World<Body>();
        world.setGravity(World.ZERO_GRAVITY);
        ball = new Body();
        BodyFixture fixture = ball.addFixture(Geometry.createCircle(0.5), 100, 0.1, 0.9);

        fixture.setRestitutionVelocity(0.0);
        // no damping
        // ball.setLinearDamping(0.8);
        // ball.setAngularDamping(0.8);
        ball.setMass(MassType.NORMAL);
        ball.setBullet(true);
        world.addBody(ball);

        // this uses simgui coordinates

        final double boundaryThickness = 0.1;
        final double fieldX = 16.541;
        final double fieldY = 8.211;

        Body wallLeft = new Body();
        fixture = wallLeft.addFixture(Geometry.createRectangle(boundaryThickness, fieldY), 1.0, 0.4, 0.3);
        fixture.setRestitutionVelocity(0.0);
        wallLeft.translate(-boundaryThickness / 2, fieldY / 2);
        wallLeft.setMass(MassType.INFINITE);
        wallLeft.setAtRestDetectionEnabled(true);
        world.addBody(wallLeft);

        Body wallRight = new Body();
        fixture = wallRight.addFixture(Geometry.createRectangle(boundaryThickness, fieldY), 1.0, 0.4, 0.3);
        fixture.setRestitutionVelocity(0.0);
        wallRight.translate(fieldX + boundaryThickness / 2, fieldY / 2);
        wallRight.setMass(MassType.INFINITE);
        wallRight.setAtRestDetectionEnabled(true);
        world.addBody(wallRight);

        Body wallTop = new Body();
        fixture = wallTop.addFixture(Geometry.createRectangle(fieldX, boundaryThickness), 1.0, 0.4, 0.3);
        fixture.setRestitutionVelocity(0.0);
        wallTop.translate(fieldX / 2, fieldY + boundaryThickness / 2);
        wallTop.setMass(MassType.INFINITE);
        wallTop.setAtRestDetectionEnabled(true);
        world.addBody(wallTop);

        Body wallBottom = new Body();
        fixture = wallBottom.addFixture(Geometry.createRectangle(fieldX, boundaryThickness), 1.0, 0.4, 0.3);
        fixture.setRestitutionVelocity(0.0);
        wallBottom.translate(fieldX / 2, -boundaryThickness / 2);
        wallBottom.setMass(MassType.INFINITE);
        wallBottom.setAtRestDetectionEnabled(true);
        world.addBody(wallBottom);

        // need the .type for rendering the field2d in sim.
        t.log(Level.INFO, "field", ".type", "Field2d");

    }

    @Override
    public void robotInit() {
        render();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // update the dyn4j sim
        world.update(0.02);
        render();
    }

    private void render() {
        // paint ball location
        Vector2 v = ball.getWorldCenter();
        double[] arr = new double[3];
        arr[0] = v.x;
        arr[1] = v.y;
        arr[2] = 0;
        t.log(Level.DEBUG, kField, kBall, arr);
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        // reset position
        ball.getTransform().identity();
        ball.getTransform().translate(1, 1);
        ball.setAtRest(false);
        // TODO: velocity units?
        ball.setLinearVelocity(new Vector2(2, 2));
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
