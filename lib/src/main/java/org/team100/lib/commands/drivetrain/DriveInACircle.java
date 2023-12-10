package org.team100.lib.commands.drivetrain;

import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.controller.DriveControllersFactory;
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
 * that point endlessly, without rotation.
 */
public class DriveInACircle extends Command {
    private static final double kDtS = 0.02;
    private static final double kRadiusM = 1.0;
    private static final double kMaxSpeed = 1.0;
    private static final double kAccel = 1.0;

    private static final Telemetry t = Telemetry.get();

    private final SwerveDriveSubsystemInterface m_swerve;
    private final HolonomicDriveController3 m_controller;

    private Pose2d m_center;
    private State100 m_rotation;
    private double m_speedRad_S;
    private double m_angleRad;

    public DriveInACircle(SwerveDriveSubsystemInterface drivetrain) {
        m_swerve = drivetrain;
        // TODO: inject the controller instead
        Identity identity = Identity.get();
        DriveControllers controllers = new DriveControllersFactory().get(identity);
        m_controller = new HolonomicDriveController3(controllers);
        if (m_swerve.get() != null)
            addRequirements(m_swerve.get());
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_swerve.getPose();
        m_center = currentPose.transformBy(new Transform2d(0, kRadiusM, GeometryUtil.kRotationZero));
        m_rotation = new State100(currentPose.getRotation().getRadians(), 0, 0);
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

        SwerveState reference = getReference(m_center, kRadiusM, m_angleRad, m_speedRad_S, accelRad_S_S, m_rotation);

        Twist2d fieldRelativeTarget = m_controller.calculate(m_swerve.getPose(), reference);
        m_swerve.driveInFieldCoords(fieldRelativeTarget);

        t.log(Level.DEBUG, "/circle/center", m_center);
        t.log(Level.DEBUG, "/circle/angle", m_angleRad);
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
