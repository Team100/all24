package org.team100.lib.commands.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestSupplierLogger;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.MockGyro;
import org.team100.lib.state.State100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class ManualWithFullStateHeadingTest {
    // a bit coarser because SimHooks.stepTiming is kinda coarse.
    private static final double kDelta = 0.01;
    private static final LoggerFactory logger = new TestSupplierLogger(new TestPrimitiveLogger());

    private Rotation2d desiredRotation = GeometryUtil.kRotationZero;

    @Test
    void testModeSwitching() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        Gyro gyro = new MockGyro();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;
        ManualWithFullStateHeading m_manualWithHeading = new ManualWithFullStateHeading(
                logger,
                swerveKinodynamics,
                gyro,
                rotationSupplier,
                new double[] { 1.0, 1.0 });
        Pose2d currentPose = GeometryUtil.kPoseZero;
        m_manualWithHeading.reset(currentPose);

        DriverControl.Velocity twist1_1 = new DriverControl.Velocity(0, 0, 0);

        FieldRelativeVelocity twistM_S = m_manualWithHeading.apply(new SwerveState(), twist1_1);
        verify(0, 0, 0, twistM_S);

        // with a non-null desired rotation we're in snap mode
        assertNotNull(m_manualWithHeading.m_goal);
        desiredRotation = null;

        twist1_1 = new DriverControl.Velocity(0, 0, 1);
        twistM_S = m_manualWithHeading.apply(new SwerveState(), twist1_1);
        // with a nonzero desired twist, we're out of snap mode
        assertNull(m_manualWithHeading.m_goal);

    }

    @Test
    void testNotSnapMode() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        Gyro gyro = new MockGyro();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;
        ManualWithFullStateHeading m_manualWithHeading = new ManualWithFullStateHeading(
                logger,
                swerveKinodynamics,
                gyro,
                rotationSupplier,
                new double[] { 1.0, 1.0 });

        Pose2d currentPose = GeometryUtil.kPoseZero;

        m_manualWithHeading.reset(currentPose);

        // no desired rotation
        desiredRotation = null;

        DriverControl.Velocity twist1_1 = new DriverControl.Velocity(0, 0, 1);

        FieldRelativeVelocity twistM_S = m_manualWithHeading.apply(
                new SwerveState(currentPose, new FieldRelativeVelocity(0, 0, 0)),
                twist1_1);

        // not in snap mode
        assertNull(m_manualWithHeading.m_goal);
        verify(0, 0, 2.828, twistM_S);

        twist1_1 = new DriverControl.Velocity(1, 0, 0);

        twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, twistM_S), twist1_1);
        assertNull(m_manualWithHeading.m_goal);
        verify(1, 0, 0, twistM_S);
    }

    @Test
    void testSnapMode() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        Gyro gyro = new MockGyro();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;
        ManualWithFullStateHeading m_manualWithHeading = new ManualWithFullStateHeading(
                logger,
                swerveKinodynamics,
                gyro,
                rotationSupplier,
                new double[] { 1.0, 1.0 });

        // facing +x
        Pose2d currentPose = GeometryUtil.kPoseZero;
        m_manualWithHeading.reset(currentPose);
        // reset means setpoint is currentpose.
        assertEquals(0, m_manualWithHeading.m_thetaSetpoint.x(), kDelta);
        assertEquals(0, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);

        // face towards +y
        desiredRotation = GeometryUtil.kRotation90;
        // no user input
        DriverControl.Velocity twist1_1 = new DriverControl.Velocity(0, 0, 0);

        FieldRelativeVelocity twistM_S = m_manualWithHeading.apply(
                new SwerveState(currentPose, new FieldRelativeVelocity(0, 0, 0)),
                twist1_1);
        // in snap mode
        assertNotNull(m_manualWithHeading.m_goal);
        // but at t0 it hasn't started yet.
        // confirm the goal is what desiredRotation says.
        assertEquals(Math.PI / 2, m_manualWithHeading.m_goal.getRadians(), kDelta);
        // setpoint is the goal
        assertEquals(1.571, m_manualWithHeading.m_thetaSetpoint.x(), kDelta);
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        verify(0, 0, 1.571, twistM_S);

        // let go of the pov to let the profile run.
        desiredRotation = null;

        // say we've rotated a little.
        currentPose = new Pose2d(0, 0, new Rotation2d(0.5));
        // cheat the setpoint for the test
        m_manualWithHeading.m_thetaSetpoint = new State100(0.5, 1);
        twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, twistM_S), twist1_1);
        // setpoint is the goal
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        assertNotNull(m_manualWithHeading.m_goal);
        // some output
        verify(0, 0, 1.071, twistM_S);

        // mostly rotated
        currentPose = new Pose2d(0, 0, new Rotation2d(1.55));
        // cheat the setpoint for the test
        m_manualWithHeading.m_thetaSetpoint = new State100(1.55, 0.2);
        twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, twistM_S), twist1_1);
        // setpoint is the goal
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        assertNotNull(m_manualWithHeading.m_goal);
        // very light pushing
        verify(0, 0, 0, twistM_S);

        // done
        currentPose = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
        m_manualWithHeading.m_thetaSetpoint = new State100(Math.PI / 2, 0);
        twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, twistM_S), twist1_1);
        assertNotNull(m_manualWithHeading.m_goal);

        // there should be no more profile to follow
        verify(0, 0, 0, twistM_S);

    }

    /** if you hold the POV the same thing should happen as above. */
    @Test
    void testSnapHeld() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        Gyro gyro = new MockGyro();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;
        ManualWithFullStateHeading m_manualWithHeading = new ManualWithFullStateHeading(
                logger,
                swerveKinodynamics,
                gyro,
                rotationSupplier,
                new double[] { 1.0, 1.0 });

        // currently facing +x
        Pose2d currentPose = GeometryUtil.kPoseZero;
        m_manualWithHeading.reset(currentPose);

        // want to face towards +y
        desiredRotation = GeometryUtil.kRotation90;
        // no dtheta

        // no stick input
        DriverControl.Velocity twist1_1 = new DriverControl.Velocity(0, 0, 0);
        FieldRelativeVelocity v = m_manualWithHeading.apply(
                new SwerveState(currentPose, new FieldRelativeVelocity(0, 0, 0)),
                twist1_1);

        // in snap mode
        assertNotNull(m_manualWithHeading.m_goal);

        // at t0 there's not much position in the profile but there is velocity
        verify(0, 0, 1.571, v);

        // say we've rotated a little.
        currentPose = new Pose2d(0, 0, new Rotation2d(0.5));

        // cheat the setpoint for the test
        m_manualWithHeading.m_thetaSetpoint = new State100(0.5, 1);
        v = m_manualWithHeading.apply(new SwerveState(currentPose, v), twist1_1);
        // the setpoint is the goal
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        assertNotNull(m_manualWithHeading.m_goal);
        verify(0, 0, 1.071, v);

        // mostly rotated, so the FB controller is calm
        currentPose = new Pose2d(0, 0, new Rotation2d(1.555));
        // cheat the setpoint for the test
        m_manualWithHeading.m_thetaSetpoint = new State100(1.555, 0.2);
        v = m_manualWithHeading.apply(new SwerveState(currentPose, v), twist1_1);
        // the setpoint is the goal
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        assertNotNull(m_manualWithHeading.m_goal);

        // pretty close to the goal now, so low output
        verify(0, 0, 0, v);

        // at the setpoint
        currentPose = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
        m_manualWithHeading.m_thetaSetpoint = new State100(Math.PI / 2, 0);
        v = m_manualWithHeading.apply(new SwerveState(currentPose, v), twist1_1);
        assertNotNull(m_manualWithHeading.m_goal);
        // there should be no more profile to follow
        verify(0, 0, 0, v);
    }

    @Test
    void testStickyHeading() {
        Experiments.instance.testOverride(Experiment.StickyHeading, true);
        MockGyro gyro = new MockGyro();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        assertEquals(2.828, swerveKinodynamics.getMaxAngleSpeedRad_S(), kDelta);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;
        ManualWithFullStateHeading m_manualWithHeading = new ManualWithFullStateHeading(
                logger,
                swerveKinodynamics,
                gyro,
                rotationSupplier,
                new double[] { 1.0, 1.0 });

        // driver rotates a bit
        DriverControl.Velocity twist1_1 = new DriverControl.Velocity(0, 0, 1);

        SwerveState currentState = new SwerveState(
                GeometryUtil.kPoseZero,
                new FieldRelativeVelocity(0, 0, 0));
        // no POV
        desiredRotation = null;

        FieldRelativeVelocity v = m_manualWithHeading.apply(currentState, twist1_1);
        // scale 1.0 input to max omega
        assertNull(m_manualWithHeading.m_goal);
        assertNull(m_manualWithHeading.m_thetaSetpoint);
        verify(0, 0, 2.828, v);

        // already going full speed:
        currentState = new SwerveState(
                GeometryUtil.kPoseZero,
                new FieldRelativeVelocity(0, 0, 2.828));
        // gyro indicates the correct speed
        gyro.rate = 2.828;
        v = m_manualWithHeading.apply(currentState, twist1_1);
        assertNull(m_manualWithHeading.m_goal);
        assertNull(m_manualWithHeading.m_thetaSetpoint);
        verify(0, 0, 2.828, v);

        // let go of the stick
        twist1_1 = new DriverControl.Velocity(0, 0, 0);
        currentState = new SwerveState(
                GeometryUtil.kPoseZero,
                new FieldRelativeVelocity(0, 0, 2.828));
        // gyro rate is still full speed.
        gyro.rate = 2.828;
        v = m_manualWithHeading.apply(currentState, twist1_1);
        // goal is the current state but at rest
        assertEquals(0.399, m_manualWithHeading.m_goal.getRadians(), kDelta);
        // setpoint is the goal
        assertEquals(0.399, m_manualWithHeading.m_thetaSetpoint.x(), kDelta);
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        // trying to stop
        verify(0, 0, -2.428, v);
    }

    @Test
    void testStickyHeading2() {
        Experiments.instance.testOverride(Experiment.StickyHeading, true);
        MockGyro gyro = new MockGyro();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        assertEquals(2.828, swerveKinodynamics.getMaxAngleSpeedRad_S(), kDelta);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;
        ManualWithFullStateHeading m_manualWithHeading = new ManualWithFullStateHeading(
                logger,
                swerveKinodynamics,
                gyro,
                rotationSupplier,
                new double[] { 1.0, 1.0 });

        // driver rotates a bit
        DriverControl.Velocity twist1_1 = new DriverControl.Velocity(0, 0, 1);

        SwerveState currentState = new SwerveState(
                GeometryUtil.kPoseZero,
                new FieldRelativeVelocity(0, 0, 0));
        // no POV
        desiredRotation = null;

        FieldRelativeVelocity v = m_manualWithHeading.apply(currentState, twist1_1);
        // scale 1.0 input to max omega
        assertNull(m_manualWithHeading.m_goal);
        assertNull(m_manualWithHeading.m_thetaSetpoint);
        verify(0, 0, 2.828, v);

        // already going full speed:
        currentState = new SwerveState(
                GeometryUtil.kPoseZero,
                new FieldRelativeVelocity(0, 0, 2.828));
        // gyro indicates the correct speed
        gyro.rate = 2.828;
        v = m_manualWithHeading.apply(currentState, twist1_1);
        assertNull(m_manualWithHeading.m_goal);
        assertNull(m_manualWithHeading.m_thetaSetpoint);
        verify(0, 0, 2.828, v);

        // let go of the stick
        twist1_1 = new DriverControl.Velocity(0, 0, 0);
        currentState = new SwerveState(
                GeometryUtil.kPoseZero,
                new FieldRelativeVelocity(0, 0, 2.828));
        // gyro rate is still full speed.
        gyro.rate = 2.828;
        v = m_manualWithHeading.apply(currentState, twist1_1);
        // goal is the current state but at rest
        assertEquals(0.399, m_manualWithHeading.m_goal.getRadians(), kDelta);
        // in this version the setpoint is the goal
        assertEquals(0.399, m_manualWithHeading.m_thetaSetpoint.x(), kDelta);
        assertEquals(0.0, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        // try very hard to stop
        verify(0, 0, -2.428, v);
    }

    /**
     * Troubleshooting the profile itself, realized the max speed was too low above
     */
    @Test
    void testProfile() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // trapezoid adapts to max actual speed
        double kRotationSpeed = 0.5;
        assertEquals(1.414, swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed, kDelta);
        assertEquals(4.243, swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed, kDelta);
        TrapezoidProfile100 m_profile = new TrapezoidProfile100(
                2.829,
                4.2,
                0.01);
        // at max heading rate
        State100 initialRaw = new State100(0, 2.828, 0);
        // goal is the same but stopped, which is an overshoot profile
        State100 goalRaw = new State100(0, 0, 0);
        State100 u = initialRaw;

        // this produces nonsensical results. using a faster profile works fine
        // but the very slow profile is wrong somehow
        // oh it's because the current speed is faster than the max speed,
        // which never happens in reality but it should do something less dumb.

        for (int i = 0; i < 100; ++i) {
            u = m_profile.calculate(0.02, u, goalRaw);
            // System.out.printf("%6.3f, %6.3f, %6.3f\n", u.x(), u.v(), u.a());
        }
    }

    private void verify(double vx, double vy, double omega, FieldRelativeVelocity v) {
        assertEquals(vx, v.x(), kDelta);
        assertEquals(vy, v.y(), kDelta);
        assertEquals(omega, v.theta(), kDelta);
    }
}
