package org.team100.commands;

import java.util.Random;

import org.team100.robot.RobotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Alliance coordinates the behavior of the member robots. */
public class Alliance {

    private final Random random = new Random();

    /**
     * Called by Command.end(), which says not to schedule anything. hm. why?
     */
    public void nextCommand(RobotSubsystem robot, Command command) {
        if (command instanceof ScoreSpeaker) {
            CommandScheduler.getInstance().schedule(new PickFromSource(this, robot));
        } else if (command instanceof ScoreAmp) {
            CommandScheduler.getInstance().schedule(new PickFromSource(this, robot));
        } else if (command instanceof PickFromSource) {
            // TODO: don't choose randomly, use a strategy and/or operator input
            if (random.nextBoolean()) {
                CommandScheduler.getInstance().schedule(new ScoreSpeaker(this, robot));
            } else {
                CommandScheduler.getInstance().schedule(new ScoreAmp(this, robot));
            }
        }
    }
}
