package org.team100.lib.commands.drivetrain.for_testing;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.logging.LoggerFactory.SwerveStateLogger;
import org.team100.lib.logging.LoggerFactory.Translation2dLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.State100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
public class DriveInACircle extends Command implements Glassy {
    private static final double kRadiusM = 1.0;
    private static final double kMaxSpeed = 0.5;
    private static final double kAccel = 0.5;

    private final SwerveDriveSubsystem m_swerve;
    private final double m_turnRatio;
    private final HolonomicFieldRelativeController m_controller;
    private final TrajectoryVisualization m_viz;

    // LOGGERS
    private final Translation2dLogger m_log_center;
    private final DoubleLogger m_log_angle;
    private final SwerveStateLogger m_log_reference;
    private final FieldRelativeVelocityLogger m_log_target;

    private Translation2d m_center;
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
            LoggerFactory parent,
            SwerveDriveSubsystem drivetrain,
            HolonomicFieldRelativeController controller,
            double turnRatio,
            TrajectoryVisualization viz) {
        LoggerFactory child = parent.child(this);
        m_log_center = child.translation2dLogger(Level.TRACE, "center");
        m_log_angle = child.doubleLogger(Level.TRACE, "angle");
        m_log_reference = child.swerveStateLogger(Level.TRACE, "reference");
        m_log_target = child.fieldRelativeVelocityLogger(Level.TRACE, "target");
        m_swerve = drivetrain;
        m_turnRatio = turnRatio;
        m_controller = controller;
        m_viz = viz;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_controller.reset();
        Pose2d currentPose = m_swerve.getState().pose();
        m_initialRotation = currentPose.getRotation().getRadians();
        m_center = getCenter(currentPose, kRadiusM);
        m_speedRad_S = 0;
        m_angleRad = 0;
        visualize();
    }

    @Override
    public void execute() {
        double accelRad_S_S = kAccel;
        m_speedRad_S += accelRad_S_S * TimedRobot100.LOOP_PERIOD_S;
        if (m_speedRad_S > kMaxSpeed) {
            accelRad_S_S = 0;
            m_speedRad_S = kMaxSpeed;
        }
        m_angleRad += m_speedRad_S * TimedRobot100.LOOP_PERIOD_S;

        SwerveState reference = getReference(
                m_center,
                kRadiusM,
                m_angleRad,
                m_speedRad_S,
                accelRad_S_S,
                m_initialRotation,
                m_turnRatio);

        FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(m_swerve.getState(), reference);
        m_swerve.driveInFieldCoords(fieldRelativeTarget);

        m_log_center.log(() -> m_center);
        m_log_angle.log(() -> m_angleRad);
        m_log_reference.log(() -> reference);
        m_log_target.log(() -> fieldRelativeTarget);
    }

    static Translation2d getCenter(Pose2d currentPose, double radiusM) {
        return currentPose.transformBy(new Transform2d(0, radiusM, GeometryUtil.kRotationZero)).getTranslation();
    }

    static SwerveState getReference(
            final Translation2d center,
            final double radiusM,
            final double angleRad,
            final double speedRad_S,
            final double accelRad_S_S,
            final double initialRotation,
            final double turnRatio) {

        State100 rotation = new State100(
                initialRotation + turnRatio * angleRad,
                turnRatio * speedRad_S,
                turnRatio * accelRad_S_S);

        double sin = Math.sin(initialRotation + angleRad);
        double cos = Math.cos(initialRotation + angleRad);
        // centripetal acceleration is omega^2*r
        // pathwise acceleration is whatever the accel parameter says
        State100 xState = new State100(
                center.getX() + sin * radiusM,
                speedRad_S * cos * radiusM,
                -1.0 * speedRad_S * speedRad_S * sin * radiusM + accelRad_S_S * cos);
        State100 yState = new State100(
                center.getY() - cos * radiusM,
                speedRad_S * sin * radiusM,
                speedRad_S * speedRad_S * cos * radiusM + accelRad_S_S * sin);
        return new SwerveState(xState, yState, rotation);
    }

    private void visualize() {
        // these poses are only used for visualization
        List<Pose2d> poses = new ArrayList<>();
        for (double angleRad = 0; angleRad < 2 * Math.PI; angleRad += 0.1) {
            SwerveState s = getReference(
                    m_center,
                    kRadiusM,
                    angleRad,
                    1.0,
                    1.0,
                    0.0,
                    0.0);
            poses.add(s.pose());
        }
        m_viz.setViz(poses);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        m_viz.clear();
    }
}
