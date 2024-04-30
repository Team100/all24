package org.team100.alliance;

import org.team100.control.ManualPilot;
import org.team100.control.auto.SpeakerCycler;
import org.team100.robot.PilotAssembly;
import org.team100.robot.RealPlayerAssembly;
import org.team100.robot.RobotAssembly;
import org.team100.robot.Source;
import org.team100.sim.Friend;
import org.team100.sim.Player;
import org.team100.sim.SimWorld;
import org.team100.strategy.Strategy2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Blue extends Alliance {
    /**
     * Use a real player robot instead of an NPC.
     */
    private static final boolean kRealPlayer = true;
    private static final Translation2d kSpeaker = new Translation2d(0, 5.548);
    private final RobotAssembly player;
    private final RobotAssembly friend1;
    private final RobotAssembly friend2;
    private final Source source;
    private final Strategy2 strategy;

    public Blue(SimWorld world) {
        Player playerBody = new Player(world);
        if (kRealPlayer) {
            player = new RealPlayerAssembly(new ManualPilot(), playerBody, kSpeaker);
        } else {
            player = new RobotAssembly(playerBody, kSpeaker);
        }
        world.addBody(playerBody);

        Friend blue1 = new Friend("blue 1", world);
        // friend1 = new RobotAssembly(blue1, kSpeaker);
        friend1 = new PilotAssembly(new SpeakerCycler(), blue1, kSpeaker);
        world.addBody(blue1);

        Friend blue2 = new Friend("blue 2", world);
        friend2 = new RobotAssembly(blue2, kSpeaker);
        world.addBody(blue2);

        source = new Source(world, 15.5, 1.0);
        strategy = new Strategy2(this, player, friend1, friend2, source, true);
    }

    public void init() {
        strategy.init();
        if (kRealPlayer)
            player.setState(2, 4, 0, 0);

    }

    public void periodic() {
        //
    }

    @Override
    public void onEnd(RobotAssembly robot, Command command) {
        strategy.onEnd(robot, command);
    }

}
