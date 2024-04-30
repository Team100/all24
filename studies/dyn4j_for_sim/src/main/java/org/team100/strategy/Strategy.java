package org.team100.strategy;

import org.team100.alliance.Alliance;
import org.team100.commands.DefendSource;
import org.team100.commands.DriveToPass;
import org.team100.commands.DriveToSource;
import org.team100.commands.DriveToAmp;
import org.team100.commands.DriveToSpeaker;
import org.team100.commands.SourceDefault;
import org.team100.robot.RobotAssembly;
import org.team100.robot.Source;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A strategy schedules commands for robots.
 * 
 * this one is the 254/1323 strategy, one passer, one scorer, one defender.
 */
public class Strategy {
    private final Alliance m_alliance;
    private final RobotAssembly m_scorer;
    private final RobotAssembly m_passer;
    private final RobotAssembly m_defender;
    private final Source m_source;
    private final boolean m_blue;

    public Strategy(
            Alliance alliance,
            RobotAssembly scorer,
            RobotAssembly passer,
            RobotAssembly defender,
            Source source,
            boolean blue) {
        m_alliance = alliance;
        m_scorer = scorer;
        m_passer = passer;
        m_defender = defender;
        m_source = source;
        m_blue = blue;
    }

    public void init() {
        if (m_scorer.isNPC()) {
            m_scorer.setState(m_blue ? 2 : 15, m_blue ? 4 : 3, 0, 0);
            CommandScheduler.getInstance().schedule(new DriveToSpeaker(m_alliance, m_scorer));
        }
        if (m_passer.isNPC()) {
            m_passer.setState(m_blue ? 1 : 15, m_blue ? 1 : 5, 0, 0);
            CommandScheduler.getInstance().schedule(new DriveToSource(m_alliance, m_passer));
        }
        if (m_defender.isNPC()) {
            m_defender.setState(m_blue ? 1 : 13, m_blue ? 4 : 7, 0, 0);
            CommandScheduler.getInstance().schedule(new DefendSource(m_alliance, m_defender));
        }
        // source just runs the default forever
        m_source.setDefaultCommand(new SourceDefault(m_source));
    }

    public void onEnd(RobotAssembly robot, Command command) {
        if (!robot.isNPC())
            return;
        if (robot == m_scorer) {
            // scorer just goes back and forth from speaker to amp
            // TODO: count amplification events
            if (command instanceof DriveToSpeaker) {
                CommandScheduler.getInstance().schedule(new DriveToAmp(m_alliance, robot));
            } else {
                CommandScheduler.getInstance().schedule(new DriveToSpeaker(m_alliance, robot));
            }
        } else if (robot == m_passer) {
            // passer goes back and forth from source to passing
            if (command instanceof DriveToPass) {
                CommandScheduler.getInstance().schedule(new DriveToSource(m_alliance, robot));
            } else {
                CommandScheduler.getInstance().schedule(new DriveToPass(m_alliance, robot));
            }
        } else if (robot == m_defender) {
            // the defender command should never return, but if it does, just do it again.
            CommandScheduler.getInstance().schedule(new DefendSource(m_alliance, robot));
        }
    }

}
