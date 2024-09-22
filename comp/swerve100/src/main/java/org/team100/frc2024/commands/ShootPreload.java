package org.team100.frc2024.commands;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.DrumShooter;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootPreload extends Command implements Glassy {
    private final SupplierLogger2 logger;
    private final Intake m_intake;
    private final SensorInterface m_sensor;
    private final FeederSubsystem m_feeder;
    private final DrumShooter m_shooter;
    private final Timer m_timer;
    private final SwerveDriveSubsystem m_drive;
    private final boolean m_isPreload;
    private final double m_pivotOverride;

    // LOGGERS
    private final DoubleSupplierLogger2 m_log_pivot_deficit;

    private boolean atVelocity = false;
    private boolean finished = false;

    public ShootPreload(
            SupplierLogger2 parent,
            SensorInterface sensor,
            DrumShooter shooter,
            Intake intake,
            FeederSubsystem feeder,
            SwerveDriveSubsystem drive,
            double pivotOverride,
            boolean isPreload) {
        logger = parent.child(this);
        m_log_pivot_deficit = logger.doubleLogger(Level.TRACE, "pivot deficit");
        m_sensor = sensor;
        m_shooter = shooter;
        m_intake = intake;
        m_feeder = feeder;
        m_drive = drive;
        m_pivotOverride = pivotOverride;
        m_isPreload = isPreload;
        m_timer = new Timer();
        addRequirements(m_intake, m_feeder, m_shooter);
    }

    public ShootPreload(
            SupplierLogger2 parent,
            SensorInterface sensor,
            DrumShooter shooter,
            Intake intake,
            FeederSubsystem feeder,
            SwerveDriveSubsystem drive,
            boolean isPreload) {
        this(parent, sensor, shooter, intake, feeder, drive, -1, isPreload);
    }

    @Override
    public void initialize() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent())
            return;

        double distance = m_drive.getState().pose().getTranslation()
                .getDistance(ShooterUtil.getSpeakerTranslation(alliance.get()));
        m_timer.reset();
        m_shooter.forward();
        // if(m_pivotOverride == -1){
        m_shooter.setAngle(ShooterUtil.getAngleRad(distance));

        m_timer.reset();
    }

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent())
            return;

        double pivotSetpoint;
        if (m_pivotOverride == -1) {
            double distance = m_drive.getState().pose().getTranslation()
                    .getDistance(ShooterUtil.getSpeakerTranslation(alliance.get()));
            pivotSetpoint = ShooterUtil.getAngleRad(distance);
        } else {
            pivotSetpoint = m_pivotOverride;
        }

        m_shooter.setAngle(pivotSetpoint);
        OptionalDouble shooterPivotPosition = m_shooter.getPivotPosition();
        if (shooterPivotPosition.isPresent()) {
            m_log_pivot_deficit.log(
                    () -> Math.abs(shooterPivotPosition.getAsDouble() - pivotSetpoint));
        }

        if (!m_sensor.getFeederSensor() && !m_sensor.getIntakeSensor()) {
            m_intake.stop();
            m_feeder.stop();
            if (shooterPivotPosition.isPresent()) {
                double error = shooterPivotPosition.getAsDouble() - pivotSetpoint;
                if (m_shooter.atVelocitySetpoint(m_isPreload)
                        && Math.abs(error) < 0.01) {
                    atVelocity = true;
                    m_timer.start();
                }
            }
        }

        if (atVelocity) {
            m_feeder.feed();
            m_intake.intake();

            if (m_timer.get() > 0.2) {
                finished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        atVelocity = false;
        finished = false;
        m_timer.stop();
        m_shooter.stop();
        m_intake.stop();
        m_feeder.stop();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public String getGlassName() {
        return "ShootPreload";
    }
}
