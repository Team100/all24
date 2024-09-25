package org.team100.lib.commands.drivetrain;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.FieldRelativeVelocityLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.SquareWave;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive back and forth, fast, while spinning, to explore the causes of veering.
 */
public class Veering extends Command implements Glassy  {
    /** translation command in m/s */
    private static final double kAmplitude = 2;
    private static final double kPeriod = 10;
    /** omega command in rad/s */
    private static final double kOmega = Math.PI;
    private final SwerveDriveSubsystem m_swerve;
    private final SquareWave m_square;
    private final Timer m_timer;
    private final FieldRelativeVelocityLogger m_log_input;

    public Veering(SupplierLogger2 parent, SwerveDriveSubsystem swerve) {
        SupplierLogger2 child = parent.child(this);
        m_swerve = swerve;
        m_square = new SquareWave(kAmplitude, kPeriod);
        m_timer = new Timer();
        addRequirements(m_swerve);
        m_log_input = child.fieldRelativeVelocityLogger(Level.TRACE, "input");
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        double dt = 0.02;
        double time = m_timer.get();
        double dx = m_square.applyAsDouble(time);
        FieldRelativeVelocity input = new FieldRelativeVelocity(dx, 0, kOmega);
        m_swerve.driveInFieldCoords(input, dt);
        m_log_input.log( () -> input);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }
}
