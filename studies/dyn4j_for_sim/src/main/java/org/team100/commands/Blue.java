package org.team100.commands;

import org.team100.robot.Source;
import org.team100.robot.RobotAssembly;
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
    private final RobotAssembly player;
    private final RobotAssembly friend1;
    private final RobotAssembly friend2;
    private final Source source;
    private final Strategy2 strategy;

    public Blue(SimWorld world) {
        player = new RobotAssembly(new Player(world), kSpeaker);
        world.addBody(player.getRobotBody());
        friend1 = new RobotAssembly(new Friend("blue 1", world), kSpeaker);
        world.addBody(friend1.getRobotBody());
        friend2 = new RobotAssembly(new Friend("blue 2", world), kSpeaker);
        world.addBody(friend2.getRobotBody());
        source = new Source(world, 15.5, 1.0);
        strategy = new Strategy2(this, player, friend1, friend2, source, true);
    }

    public void init() {
        strategy.init();
        if (kRealPlayer) {
            // override the scheduled command
            Command scheduled = CommandScheduler.getInstance().requiring(player.getRobotSubsystem());
            System.out.printf("scheduled %s\n", scheduled);
            player.getRobotSubsystem().setDefaultCommand(new PlayerDefault(player));
            // this triggers onEnd so there's a condition for it below. :(
            if (scheduled != null)
                CommandScheduler.getInstance().cancel(scheduled);
        }
    }

    public void periodic() {
        //
    }

    @Override
    public void onEnd(RobotAssembly robot, Command command) {
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
