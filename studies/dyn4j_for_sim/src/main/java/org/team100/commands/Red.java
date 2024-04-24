package org.team100.commands;

import java.util.Random;

import org.team100.robot.RobotSubsystem;
import org.team100.sim.Foe;
import org.team100.sim.SimWorld;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Red extends Alliance {
    private final Random random = new Random();

    private final RobotSubsystem scorer;
    private final RobotSubsystem passer;
    private final RobotSubsystem defender;

    public Red(SimWorld world) {
        scorer = new RobotSubsystem(new Foe("red scorer", world));
        world.addBody(scorer.getRobotBody());
        scorer.setDefaultCommand(new NonplayerDefault(scorer));

        passer = new RobotSubsystem(new Foe("red 2", world));
        world.addBody(passer.getRobotBody());
        passer.setDefaultCommand(new NonplayerDefault(passer));

        defender = new RobotSubsystem(new Foe("red 3", world));
        world.addBody(defender.getRobotBody());
        defender.setDefaultCommand(new NonplayerDefault(defender));
    }

    public void init() {
        scorer.setState(15, 3, -4, 0);
        passer.setState(15, 5, -4, -4);
        defender.setState(13, 7, -4, 4);
        CommandScheduler.getInstance().schedule(new ScoreSpeaker(this, scorer));
        CommandScheduler.getInstance().schedule(new PickFromSource(this, passer));
        CommandScheduler.getInstance().schedule(new DefendSource(this, defender));
    }

    /** Think about what to do. */
    public void periodic() {
        //
    }

    /**
     * Called by Command.end(), to signal the alliance to choose a new command.
     */
    @Override
    public void onEnd(RobotSubsystem robot, Command command) {
        if (robot == scorer) {
            // scorer just goes back and forth from speaker to amp
            // TODO: count amplification events
            if (command instanceof ScoreSpeaker) {
                CommandScheduler.getInstance().schedule(new ScoreAmp(this, robot));
            } else {
                CommandScheduler.getInstance().schedule(new ScoreSpeaker(this, robot));
            }
        } else if (robot == passer) {
            // passer goes back and forth from source to passing
            if (command instanceof Pass) {
                CommandScheduler.getInstance().schedule(new PickFromSource(this, robot));
            } else {
                CommandScheduler.getInstance().schedule(new Pass(this, robot));
            }
        } else if (robot == defender) {
            // the defender command should never return, but if it does, just do it again.
            CommandScheduler.getInstance().schedule(new DefendSource(this, robot));
        } else {
            // this is the old logic; it should never be called.

            if (command instanceof ScoreSpeaker) {
                CommandScheduler.getInstance().schedule(new PickFromSource(this, robot));
            } else if (command instanceof ScoreAmp) {
                CommandScheduler.getInstance().schedule(new PickFromSource(this, robot));
            } else if (command instanceof Pass) {
                CommandScheduler.getInstance().schedule(new PickFromSource(this, robot));
            } else if (command instanceof PickFromSource) {
                // TODO: don't choose randomly, use a strategy and/or operator input
                if (random.nextBoolean()) {
                    CommandScheduler.getInstance().schedule(new ScoreSpeaker(this, robot));
                } else {
                    // CommandScheduler.getInstance().schedule(new ScoreAmp(this, robot));
                    CommandScheduler.getInstance().schedule(new Pass(this, robot));
                }
            }
        }
    }
}
