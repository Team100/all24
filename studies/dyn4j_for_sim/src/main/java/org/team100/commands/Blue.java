package org.team100.commands;

import org.team100.robot.Source;
import org.team100.robot.RobotSubsystem;
import org.team100.sim.Friend;
import org.team100.sim.Player;
import org.team100.sim.SimWorld;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Blue extends Alliance {
    /**
     * Use a real player robot instead of an NPC.
     */
    private static final boolean kRealPlayer = false;
    private static final Translation2d kSpeaker = new Translation2d(0, 5.548);
    private final RobotSubsystem player;
    private final RobotSubsystem friend1;
    private final RobotSubsystem friend2;
    private final Source source;
    private final Strategy2 strategy;

    public Blue(SimWorld world) {
        player = new RobotSubsystem(new Player(world), kSpeaker);
        world.addBody(player.getRobotBody());
        friend1 = new RobotSubsystem(new Friend("blue 1", world), kSpeaker);
        world.addBody(friend1.getRobotBody());
        friend2 = new RobotSubsystem(new Friend("blue 2", world), kSpeaker);
        world.addBody(friend2.getRobotBody());
        source = new Source(world, 15.5, 1.0);
        strategy = new Strategy2(this, player, friend1, friend2, source, true);
    }

    public void init() {
        strategy.init();
        if (kRealPlayer) {
            // override the scheduled command
            Command scheduled = CommandScheduler.getInstance().requiring(player);
            System.out.printf("scheduled %s\n", scheduled);
            player.setDefaultCommand(new PlayerDefault(player));
            // this triggers onEnd so there's a condition for it below. :(
            if (scheduled != null)
                CommandScheduler.getInstance().cancel(scheduled);
        }
    }

    public void periodic() {
        //
    }

    @Override
    public void onEnd(RobotSubsystem robot, Command command) {
        if (kRealPlayer) {
            if (robot == player) {
                System.out.println("skip rescheduling for player");
                // let the player continue with the default
                return;
            }
        }
        strategy.onEnd(robot, command);
    }

}
