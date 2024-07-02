package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.geometry.Vector2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

class SlipperyTireUtilTest {
    private static final double kDelta = 0.001;

    // radius is sqrt(2)/2
    SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
            new Translation2d(0.5, 0.5),
            new Translation2d(0.5, -0.5),
            new Translation2d(-0.5, 0.5),
            new Translation2d(-0.5, -0.5));

    @Test
    void testMotionless() {
        // starting point
        Pose2d pose0 = new Pose2d();
        // ending point, same
        Pose2d pose1 = new Pose2d();
        double dtSeconds = 0.02;
        // calculate the difference
        Twist2d twist = pose0.log(pose1);
        assertEquals(0, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0, twist.dtheta, kDelta);
        ChassisSpeeds speeds = new ChassisSpeeds(
                twist.dx / dtSeconds,
                twist.dy / dtSeconds,
                twist.dtheta / dtSeconds);

        // state is a velocity at an angle
        // i think the idea is that this angle would be fixed
        // during the whole time?
        SwerveModuleState100[] states = kinematics.toSwerveModuleStates(speeds);
        check(0, 0, states[0]);
        check(0, 0, states[1]);
        check(0, 0, states[2]);
        check(0, 0, states[3]);

        // position is also an angle, maybe ignore it
        SwerveModulePosition[] position0 = new SwerveModulePosition[] {
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d())
        };
        check(0, 0, position0[0], kDelta);
        check(0, 0, position0[1], kDelta);
        check(0, 0, position0[2], kDelta);
        check(0, 0, position0[3], kDelta);

        // all these angles should be somehow consistent
        // maybe just use the velocity angle.
        SwerveModulePosition[] position1 = new SwerveModulePosition[4];
        for (int i = 0; i < position1.length; ++i) {
            position1[i] = new SwerveModulePosition(
                    position0[i].distanceMeters + states[i].speedMetersPerSecond * dtSeconds,
                    states[i].angle);
        }
        check(0, 0, position1[0], kDelta);
        check(0, 0, position1[1], kDelta);
        check(0, 0, position1[2], kDelta);
        check(0, 0, position1[3], kDelta);

        SwerveModulePosition[] modulePositionDelta = DriveUtil.modulePositionDelta(
                new SwerveDriveWheelPositions(position0),
                new SwerveDriveWheelPositions(position1));

        Twist2d twist2 = kinematics.toTwist2d(modulePositionDelta);
        assertEquals(0, twist2.dx, kDelta);
        assertEquals(0, twist2.dy, kDelta);
        assertEquals(0, twist2.dtheta, kDelta);

        Vector2d[] vecs = kinematics.pos2vec(modulePositionDelta);
        check(0, 0, vecs[0]);
        check(0, 0, vecs[1]);
        check(0, 0, vecs[2]);
        check(0, 0, vecs[3]);

        // using the util
        Vector2d[] corners = SlipperyTireUtil.cornerDeltas(kinematics, pose0, pose1);
        check(0, 0, corners[0]);
        check(0, 0, corners[1]);
        check(0, 0, corners[2]);
        check(0, 0, corners[3]);

    }

    @Test
    void testLinear() {
        // starting point
        Pose2d pose0 = new Pose2d();
        // ending point 1m/s +x
        Pose2d pose1 = new Pose2d(new Translation2d(0.02, 0), new Rotation2d());
        double dtSeconds = 0.02;
        // calculate the difference
        Twist2d twist = pose0.log(pose1);
        assertEquals(0.02, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0, twist.dtheta, kDelta);
        ChassisSpeeds speeds = new ChassisSpeeds(
                twist.dx / dtSeconds,
                twist.dy / dtSeconds,
                twist.dtheta / dtSeconds);

        // state is a velocity at an angle
        // i think the idea is that this angle would be fixed
        // during the whole time?
        SwerveModuleState100[] states = kinematics.toSwerveModuleStates(speeds);
        check(1, 0, states[0]);
        check(1, 0, states[1]);
        check(1, 0, states[2]);
        check(1, 0, states[3]);

        // position is also an angle, maybe ignore it
        SwerveModulePosition[] position0 = new SwerveModulePosition[] {
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d())
        };
        check(0, 0, position0[0], kDelta);
        check(0, 0, position0[1], kDelta);
        check(0, 0, position0[2], kDelta);
        check(0, 0, position0[3], kDelta);

        // all these angles should be somehow consistent
        // maybe just use the velocity angle.
        SwerveModulePosition[] position1 = new SwerveModulePosition[4];
        for (int i = 0; i < position1.length; ++i) {
            position1[i] = new SwerveModulePosition(
                    position0[i].distanceMeters + states[i].speedMetersPerSecond * dtSeconds,
                    states[i].angle);
        }
        check(0.02, 0, position1[0], kDelta);
        check(0.02, 0, position1[1], kDelta);
        check(0.02, 0, position1[2], kDelta);
        check(0.02, 0, position1[3], kDelta);

        SwerveModulePosition[] modulePositionDelta = DriveUtil.modulePositionDelta(
                new SwerveDriveWheelPositions(position0),
                new SwerveDriveWheelPositions(position1));

        Twist2d twist2 = kinematics.toTwist2d(modulePositionDelta);
        assertEquals(0.02, twist2.dx, kDelta);
        assertEquals(0, twist2.dy, kDelta);
        assertEquals(0, twist2.dtheta, kDelta);

        Vector2d[] vecs = kinematics.pos2vec(modulePositionDelta);
        check(0.02, 0, vecs[0]);
        check(0.02, 0, vecs[1]);
        check(0.02, 0, vecs[2]);
        check(0.02, 0, vecs[3]);

        // using the util
        Vector2d[] corners = SlipperyTireUtil.cornerDeltas(kinematics, pose0, pose1);
        check(0.02, 0, corners[0]);
        check(0.02, 0, corners[1]);
        check(0.02, 0, corners[2]);
        check(0.02, 0, corners[3]);

    }

    @Test
    void testSpin() {
        // starting point
        Pose2d pose0 = new Pose2d();
        // ending point 1m/s +x
        Pose2d pose1 = new Pose2d(new Translation2d(), new Rotation2d(0.02));
        double dtSeconds = 0.02;
        // calculate the difference
        Twist2d twist = pose0.log(pose1);
        assertEquals(0, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0.02, twist.dtheta, kDelta);
        ChassisSpeeds speeds = new ChassisSpeeds(
                twist.dx / dtSeconds,
                twist.dy / dtSeconds,
                twist.dtheta / dtSeconds);

        // speed
        // 0.02 radians * radius of sqrt(2)/2 / 0.02 sec = sqrt(2)/2
        SwerveModuleState100[] states = kinematics.toSwerveModuleStates(speeds);
        check(0.707, 135, states[0]);
        check(0.707, 45, states[1]);
        check(0.707, -135, states[2]);
        check(0.707, -45, states[3]);

        // position is also an angle, maybe ignore it
        // for this one just set the starting position to be right
        SwerveModulePosition[] position0 = new SwerveModulePosition[] {
                new SwerveModulePosition(0, Rotation2d.fromDegrees(135)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(45)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(-135)),
                new SwerveModulePosition(0, Rotation2d.fromDegrees(-45))
        };
        check(0, 135, position0[0], kDelta);
        check(0, 45, position0[1], kDelta);
        check(0, -135, position0[2], kDelta);
        check(0, -45, position0[3], kDelta);

        // postion = speed * dt
        // 0.707 * 0.02 = 0.014
        SwerveModulePosition[] position1 = new SwerveModulePosition[4];
        for (int i = 0; i < position1.length; ++i) {
            position1[i] = new SwerveModulePosition(
                    position0[i].distanceMeters + states[i].speedMetersPerSecond * dtSeconds,
                    states[i].angle);
        }
        check(0.014, 135, position1[0], kDelta);
        check(0.014, 45, position1[1], kDelta);
        check(0.014, -135, position1[2], kDelta);
        check(0.014, -45, position1[3], kDelta);

        SwerveModulePosition[] modulePositionDelta = DriveUtil.modulePositionDelta(
                new SwerveDriveWheelPositions(position0),
                new SwerveDriveWheelPositions(position1));

        Twist2d twist2 = kinematics.toTwist2d(modulePositionDelta);
        assertEquals(0, twist2.dx, kDelta);
        assertEquals(0, twist2.dy, kDelta);
        assertEquals(0.02, twist2.dtheta, kDelta);

        // 0.014 on diagonal = 0.01 on each axis
        Vector2d[] vecs = kinematics.pos2vec(modulePositionDelta);
        check(-0.01, 0.01, vecs[0]);
        check(0.01, 0.01, vecs[1]);
        check(-0.01, -0.01, vecs[2]);
        check(0.01, -0.01, vecs[3]);

        // using the util
        Vector2d[] corners = SlipperyTireUtil.cornerDeltas(kinematics, pose0, pose1);
        check(-0.01, 0.01, corners[0]);
        check(0.01, 0.01, corners[1]);
        check(-0.01, -0.01, corners[2]);
        check(0.01, -0.01, corners[3]);
    }

    @Test
    void testLaunch() {
        // initially at rest
        Pose2d pose0 = new Pose2d();
        Pose2d pose1 = new Pose2d();
        final double dtSeconds = 0.02;
        // corners are not moving
        Vector2d[] corners = SlipperyTireUtil.cornerDeltas(kinematics, pose0, pose1);
        // but now there's some odometry showing 1 m/s (!)
        SwerveModulePosition[] deltas = new SwerveModulePosition[] {
                new SwerveModulePosition(0.02, new Rotation2d()),
                new SwerveModulePosition(0.02, new Rotation2d()),
                new SwerveModulePosition(0.02, new Rotation2d()),
                new SwerveModulePosition(0.02, new Rotation2d())
        };
        Tire t = new Tire(10.0, 0.1);
        SlipperyTireUtil u = new SlipperyTireUtil(t);
        SwerveModulePosition[] adjusted = u.adjust(corners, dtSeconds, deltas, dtSeconds);
        // we wanted to go 1 m/s but this is 50m/s/s of accel
        // which is way beyond the 10 m/s/s limit.
        // so you get 10 m/s/s for 0.02 s so you move 10*0.02*0.02=0.004.
        check(0.004, 0, adjusted[0], 1e-6);
        check(0.004, 0, adjusted[1], 1e-6);
        check(0.004, 0, adjusted[2], 1e-6);
        check(0.004, 0, adjusted[3], 1e-6);
    }

    @Test
    void testStop() {
        // initially moving 1 m/s +x
        Pose2d pose0 = new Pose2d();
        Pose2d pose1 = new Pose2d(new Translation2d(0.02, 0), new Rotation2d());
        final double dtSeconds = 0.02;
        Vector2d[] corners = SlipperyTireUtil.cornerDeltas(kinematics, pose0, pose1);
        // now lock the wheels
        SwerveModulePosition[] deltas = new SwerveModulePosition[] {
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d())
        };

        Tire t = new Tire(10.0, 0.1);
        SlipperyTireUtil u = new SlipperyTireUtil(t);
        SwerveModulePosition[] adjusted = u.adjust(corners, dtSeconds, deltas, dtSeconds);
        // as above we are trying for 50 m/s/s but we get 10.
        // so slide along, instead of 0.02 go 0.016
        check(0.016, 0, adjusted[0], 1e-6);
        check(0.016, 0, adjusted[1], 1e-6);
        check(0.016, 0, adjusted[2], 1e-6);
        check(0.016, 0, adjusted[3], 1e-6);
    }

    void check(
            double meters,
            double degrees,
            SwerveModulePosition position,
            double delta) {
        assertEquals(meters, position.distanceMeters, delta);
        assertEquals(degrees, position.angle.getDegrees(), delta);
    }

    void check(
            double metersPerSecond,
            double degrees,
            SwerveModuleState100 state) {
        assertEquals(metersPerSecond, state.speedMetersPerSecond, kDelta);
        assertEquals(degrees, state.angle.getDegrees(), kDelta);
    }

    void check(
            double x,
            double y,
            Vector2d v) {
        assertEquals(x, v.getX(), kDelta);
        assertEquals(y, v.getY(), kDelta);
    }

}
