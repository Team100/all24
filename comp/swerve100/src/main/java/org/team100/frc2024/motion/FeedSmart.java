package org.team100.frc2024.motion;

import org.team100.frc2024.motion.shooter.DrumShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FeedSmart extends Command {
    private final FeederSubsystem m_feeder;
    private final DrumShooter m_shooter;
    private final Timer m_timer;

    private boolean atSetpoint = false;
    private boolean finished = false;

    public FeedSmart(FeederSubsystem feeder, DrumShooter shooter) {
        m_feeder = feeder;
        m_shooter = shooter;
        m_timer = new Timer();
        addRequirements(m_feeder);
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        if (m_shooter.atVelocitySetpoint()) {
            m_feeder.feed();
            m_timer.start();
            atSetpoint = true;
        }

        if (atSetpoint && (m_timer.get() > 1)) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
