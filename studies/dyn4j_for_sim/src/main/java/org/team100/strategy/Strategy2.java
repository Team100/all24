package org.team100.strategy;

import java.util.Random;

import org.team100.alliance.Alliance;
import org.team100.commands.DefendSource;
import org.team100.commands.Pass;
import org.team100.commands.PickFromSource;
import org.team100.commands.ScoreAmp;
import org.team100.commands.DriveToSpeaker;
import org.team100.commands.SourceDefault;
import org.team100.robot.RobotAssembly;
import org.team100.robot.Source;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This is the 1690 strategy: two cyclers, one defender.
 */
public class Strategy2 {
    private final Alliance m_alliance;
    private final RobotAssembly m_player;
    private final RobotAssembly m_friend1;
    private final RobotAssembly m_friend2;
    private final Source m_source;
    private final boolean m_blue;

    private final Random random = new Random();

    public Strategy2(
            Alliance alliance,
            RobotAssembly player,
            RobotAssembly friend1,
            RobotAssembly friend2,
            Source source,
            boolean blue) {
        m_alliance = alliance;
        m_player = player;
        m_friend1 = friend1;
        m_friend2 = friend2;
        m_source = source;
        m_blue = blue;
    }

    /** Does not affect instances of RealPlayerAssembly. */
    public void init() {
        if (m_player.isNPC()) {
            m_player.setState(m_blue ? 2 : 15, m_blue ? 4 : 3, 0, 0);
            CommandScheduler.getInstance().schedule(new DriveToSpeaker(m_alliance, m_player));
        }
        if (m_friend1.isNPC()) {
            m_friend1.setState(m_blue ? 1 : 15, m_blue ? 1 : 5, 0, 0);
            CommandScheduler.getInstance().schedule(new DriveToSpeaker(m_alliance, m_friend1));
        }
        if (m_friend2.isNPC()) {
            m_friend2.setState(m_blue ? 1 : 13, m_blue ? 4 : 7, 0, 0);
            CommandScheduler.getInstance().schedule(new DefendSource(m_alliance, m_friend2));
        }
        // source just runs the default forever
        m_source.setDefaultCommand(new SourceDefault(m_source));
    }

    public void onEnd(RobotAssembly robot, Command command) {
        if (!robot.isNPC())
            return;
        System.out.printf("on end %s %s\n", robot.getName(), command.getName());
        if (command instanceof DriveToSpeaker) {
            CommandScheduler.getInstance().schedule(new PickFromSource(m_alliance, robot));
        } else if (command instanceof ScoreAmp) {
            CommandScheduler.getInstance().schedule(new PickFromSource(m_alliance, robot));
        } else if (command instanceof Pass) {
            CommandScheduler.getInstance().schedule(new PickFromSource(m_alliance, robot));
        } else if (command instanceof PickFromSource) {
            // TODO: don't choose randomly, use a strategy and/or operator input
            if (random.nextBoolean()) {
                CommandScheduler.getInstance().schedule(new DriveToSpeaker(m_alliance, robot));
            } else {
                // CommandScheduler.getInstance().schedule(new ScoreAmp(m_alliance, robot));
                CommandScheduler.getInstance().schedule(new Pass(m_alliance, robot));
            }
        }
    }

}
