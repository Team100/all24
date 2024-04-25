package org.team100.commands;

import org.team100.robot.Source;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SourceDefault extends Command {
    /** How often to add a note. */
    private static final double kPeriodS = 1;

    private final Source m_humanPlayer;
    private final Timer m_timer;

    public SourceDefault(Source source) {
        m_humanPlayer = source;
        m_timer = new Timer();
        addRequirements(source);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        if (m_timer.advanceIfElapsed(kPeriodS)) {
            m_humanPlayer.feed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }
}
