package org.team100.frc2024.commands;

import java.util.Optional;

import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootPreload extends Command {
    private static final Telemetry t = Telemetry.get();

    private final Intake m_intake;
    private final SensorInterface m_sensor;
    private final FeederSubsystem m_feeder;
    private final Shooter m_shooter;
    private final Timer m_timer;
    private final SwerveDriveSubsystem m_drive;
    private final boolean m_isPreload;
    private final double m_pivotOverride;

    private boolean atVelocity = false;
    private boolean finished = false;

    public ShootPreload(
            SensorInterface sensor,
            Shooter shooter, Intake intake,
            FeederSubsystem feeder,
            SwerveDriveSubsystem drive,
            double pivotOverride,
            boolean isPreload) {
        m_intake = intake;
        m_sensor = sensor;
        m_feeder = feeder;
        m_shooter = shooter;
        m_timer = new Timer();
        m_drive = drive;
        m_pivotOverride = pivotOverride;
        m_isPreload = isPreload;
        addRequirements(m_intake, m_feeder, m_shooter);
    }

    public ShootPreload(
            SensorInterface sensor,
            Shooter shooter,
            Intake intake,
            FeederSubsystem feeder,
            SwerveDriveSubsystem drive,
            boolean isPreload) {
        m_intake = intake;
        m_sensor = sensor;
        m_feeder = feeder;
        m_shooter = shooter;
        m_timer = new Timer();
        m_drive = drive;
        m_pivotOverride = -1;
        m_isPreload = isPreload;

        addRequirements(m_intake, m_feeder, m_shooter);
    }

    @Override
    public void initialize() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent())
            return;

        t.log(Level.DEBUG, "ShootSmart", "command state", "initialize");

        double distance = m_drive.getPose().getTranslation()
                .getDistance(ShooterUtil.getSpeakerTranslation(alliance.get()));
        m_timer.reset();
        m_shooter.forward();
        // if(m_pivotOverride == -1){
        m_shooter.setAngle(ShooterUtil.getAngleRad(distance));

        m_timer.reset();
    }

    @Override
    public void execute() {
        t.log(Level.DEBUG, "ShootSmart", "command state", "end");
        t.log(Level.DEBUG, "ShootSmart", "END", 0);
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent())
            return;

        double pivotSetpoint;
        if (m_pivotOverride == -1) {
            double distance = m_drive.getPose().getTranslation()
                    .getDistance(ShooterUtil.getSpeakerTranslation(alliance.get()));
            pivotSetpoint = ShooterUtil.getAngleRad(distance);
        } else {
            pivotSetpoint = m_pivotOverride;
        }

        m_shooter.setAngle(pivotSetpoint);

        t.log(Level.DEBUG, "ShootSmart", "PIVOT DEFECIT", Math.abs(m_shooter.getPivotPosition() - pivotSetpoint));

        if (!m_sensor.getFeederSensor() && !m_sensor.getIntakeSensor()) {
            m_intake.stop();
            m_feeder.stop();

            double error = m_shooter.getPivotPosition() - pivotSetpoint;
            if (m_shooter.atVelocitySetpoint(m_isPreload)
                    && Math.abs(error) < 0.01) {
                atVelocity = true;
                m_timer.start();
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
        t.log(Level.DEBUG, "ShootSmart", "command state", "end");
        t.log(Level.DEBUG, "ShootSmart", "command state", "end");
        t.log(Level.DEBUG, "ShootSmart", "END", 1);
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
}
