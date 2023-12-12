package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.State100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Define a center point 1m to the left of the starting position, and circle
 * that point endlessly, with rotation. You can specify the ratio of rotation
 * between the robot itself and the circle it is traveling, to make useful
 * figures for calibration.
 * 
 * This trochoid desmos is handy for choosing ratios:
 * 
 * https://www.desmos.com/calculator/3plby3pgqv
 */
public class DriveInACircle extends Command {
    private static final double kDtS = 0.02;
    private static final double kRadiusM = 1.0;
    private static final double kMaxSpeed = 0.5;
    private static final double kAccel = 0.5;

    private static final Telemetry t = Telemetry.get();

    private final SwerveDriveSubsystemInterface m_swerve;
    private double m_turnRatio;
    private final HolonomicDriveController3 m_controller;

    private Pose2d m_center;
    private double m_initialRotation;
    private double m_speedRad_S;
    private double m_angleRad;

    /**
     * @param turnRatio How to rotate the drivetrain.
     *                  Use 0 for no rotation.
     *                  Use 1 to fix the aiming point.
     *                  Larger positive numbers produce epitrochoids, i.e.
     *                  "flowers."
     *                  Negative numbers produce hypotrochoids, i.e. "stars," e.g.
     *                  use -3 to produce a four-pointed star called an Astroid:
     *                  https://en.wikipedia.org/wiki/Astroid.
     * 
     */
    public DriveInACircle(
            SwerveDriveSubsystemInterface drivetrain,
            HolonomicDriveController3 controller,
            double turnRatio) {
        m_swerve = drivetrain;
        m_turnRatio = turnRatio;
        m_controller = controller;
        if (m_swerve.get() != null)
            addRequirements(m_swerve.get());
    }

    @Override
    public void initialize() {
        m_controller.reset();
        Pose2d currentPose = m_swerve.getPose();
        m_center = currentPose.transformBy(new Transform2d(0, kRadiusM, GeometryUtil.kRotationZero));
        m_initialRotation = currentPose.getRotation().getRadians();
        m_speedRad_S = 0;
        m_angleRad = 0;
    }

    @Override
    public void execute() {
        double accelRad_S_S = kAccel;
        m_speedRad_S += accelRad_S_S * kDtS;
        if (m_speedRad_S > kMaxSpeed) {
            accelRad_S_S = 0;
            m_speedRad_S = kMaxSpeed;
        }
        m_angleRad += m_speedRad_S * kDtS;

        State100 rotation = new State100(
                m_initialRotation + m_turnRatio * m_angleRad,
                m_turnRatio * m_speedRad_S,
                m_turnRatio * accelRad_S_S);

        SwerveState reference = getReference(m_center, kRadiusM, m_angleRad, m_speedRad_S, accelRad_S_S, rotation);

        Twist2d fieldRelativeTarget = m_controller.calculate(m_swerve.getPose(), reference);
        m_swerve.driveInFieldCoords(fieldRelativeTarget);

        t.log(Level.DEBUG, "/circle/center", m_center);
        t.log(Level.DEBUG, "/circle/angle", m_angleRad);
        t.log(Level.DEBUG, "/circle/reference", reference);
        t.log(Level.DEBUG, "/circle/target", fieldRelativeTarget);
    }

    static Pose2d getCenter(Pose2d currentPose, double radiusM) {
        return currentPose.transformBy(new Transform2d(0, radiusM, GeometryUtil.kRotationZero));
    }

    static SwerveState getReference(
            Pose2d center,
            double radiusM,
            double angleRad,
            double speedRad_S,
            double accelRad_S_S,
            State100 rotation) {
        // centripetal acceleration is omega^2*r
        // pathwise acceleration is whatever the accel parameter says
        State100 xState = new State100(
                center.getX() + Math.sin(angleRad) * radiusM,
                speedRad_S * Math.cos(angleRad) * radiusM,
                -1.0 * speedRad_S * speedRad_S * Math.sin(angleRad) * radiusM + accelRad_S_S * Math.cos(angleRad));
        State100 yState = new State100(
                center.getY() - Math.cos(angleRad) * radiusM,
                speedRad_S * Math.sin(angleRad) * radiusM,
                speedRad_S * speedRad_S * Math.cos(angleRad) * kRadiusM + accelRad_S_S * Math.sin(angleRad));
        t.log(Level.DEBUG, "/circle/x_state", xState);
        t.log(Level.DEBUG, "/circle/y_state", yState);
        return new SwerveState(xState, yState, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }
}
