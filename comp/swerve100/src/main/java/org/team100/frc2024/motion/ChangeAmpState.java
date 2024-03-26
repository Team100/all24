package org.team100.frc2024.motion;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.AmpState100;
import org.team100.frc2024.motion.amp.AmpSubsystem;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj2.command.Command;

public class ChangeAmpState extends Command {
    private static final Telemetry t = Telemetry.get();

    private final AmpState100 m_state;
    private final AmpSubsystem m_amp;

    public ChangeAmpState(AmpState100 state, AmpSubsystem amp) {
        m_state = state;
        m_amp = amp;
    }

    @Override
    public void initialize() {
        t.log(Level.DEBUG, "ChangeAmpState", "command state", "initialize");
        m_amp.reset();
        RobotState100.changeAmpState(m_state);
    }

    @Override
    public void execute() {
        t.log(Level.DEBUG, "ChangeAmpState", "command state", "execute");
    }

    @Override
    public void end(boolean interrupted) {
        t.log(Level.DEBUG, "ChangeAmpState", "command state", "end");
        RobotState100.changeAmpState(AmpState100.NONE);
    }
}
