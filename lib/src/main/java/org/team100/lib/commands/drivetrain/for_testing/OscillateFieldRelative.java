package org.team100.lib.commands.drivetrain.for_testing;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.ParabolicWave;
import org.team100.lib.util.SquareWave;
import org.team100.lib.util.TriangleWave;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Like {@link Oscillate} but field-relative. */
public class OscillateFieldRelative extends Command implements Glassy {
    private static final double kAccel = 1;
    private static final double kMaxSpeed = 1;
    private static final double kPeriod = 4 * kMaxSpeed / kAccel;

    private final SwerveDriveSubsystem m_swerve;
    private final SquareWave m_square;
    private final TriangleWave m_triangle;
    private final ParabolicWave m_parabola;
    private final Timer m_timer;

    // LOGGERS
    private final DoubleSupplierLogger2 m_log_period;
    private final DoubleSupplierLogger2 m_log_time;
    private final DoubleSupplierLogger2 m_log_setpoint_accel;
    private final DoubleSupplierLogger2 m_log_setpoint_speed;
    private final DoubleSupplierLogger2 m_log_setpoint_position;
    private final DoubleSupplierLogger2 m_log_measurement_speed;
    private final DoubleSupplierLogger2 m_log_measurement_position;

    SwerveState m_initial;

    public OscillateFieldRelative(SupplierLogger2 parent, SwerveDriveSubsystem swerve) {
        SupplierLogger2 child = parent.child(this);
        m_swerve = swerve;
        m_square = new SquareWave(kAccel, kPeriod);
        m_triangle = new TriangleWave(kMaxSpeed, kPeriod);
        m_parabola = new ParabolicWave(kMaxSpeed * kPeriod / 4, kPeriod);
        m_timer = new Timer();
        addRequirements(m_swerve);
        m_log_period = child.doubleLogger(Level.DEBUG, "period");
        m_log_time = child.doubleLogger(Level.DEBUG, "time");
        m_log_setpoint_accel = child.doubleLogger(Level.DEBUG, "setpoint/accel");
        m_log_setpoint_speed = child.doubleLogger(Level.DEBUG, "setpoint/speed");
        m_log_setpoint_position = child.doubleLogger(Level.DEBUG, "setpoint/position");
        m_log_measurement_speed = child.doubleLogger(Level.DEBUG, "measurement/speed");
        m_log_measurement_position = child.doubleLogger(Level.DEBUG, "measurement/position");
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_initial = m_swerve.getState();
    }

    @Override
    public void execute() {
        double time = m_timer.get();
        double accelM_S_S = m_square.applyAsDouble(time);
        double speedM_S = m_triangle.applyAsDouble(time);
        double positionM = m_parabola.applyAsDouble(time);

        m_swerve.driveInFieldCoords(new FieldRelativeVelocity(speedM_S, 0, 0));

        m_log_period.log(() -> kPeriod);
        m_log_time.log(() -> time);
        m_log_setpoint_accel.log(() -> accelM_S_S);
        m_log_setpoint_speed.log(() -> speedM_S);
        m_log_setpoint_position.log(() -> positionM);
        SwerveState swerveState = m_swerve.getState();
        m_log_measurement_speed.log(() -> swerveState.x().v());
        m_log_measurement_position.log(() -> swerveState.x().x() - m_initial.x().x());
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

}
