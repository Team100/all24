package org.team100.alliance;

import org.team100.commands.SourceDefault;
import org.team100.control.ManualPilot;
import org.team100.control.auto.AmpCycler;
import org.team100.control.auto.Defender;
import org.team100.control.auto.SpeakerCycler;
import org.team100.robot.PilotAssembly;
import org.team100.robot.RealPlayerAssembly;
import org.team100.robot.RobotAssembly;
import org.team100.robot.Source;
import org.team100.sim.Friend;
import org.team100.sim.Player;
import org.team100.sim.SimWorld;

import edu.wpi.first.math.geometry.Translation2d;

public class Blue implements Alliance {
    /**
     * Use a real player robot instead of an NPC.
     */
    private static final boolean kRealPlayer = false;
    private final RobotAssembly player;
    private final RobotAssembly friend1;
    private final RobotAssembly friend2;
    private final Source source;

    public Blue(SimWorld world) {
        if (kRealPlayer) {
            player = new RealPlayerAssembly(
                    x -> new ManualPilot(),
                    new Player(world, false),
                    false);
            // use the pilot assembly with manual control, to test the buttons.
            // player = new PilotAssembly(x -> new ManualPilot(), playerBody, kSpeaker);
        } else {
            player = new PilotAssembly(
                    x -> new AmpCycler(x.getDrive(), x.getCamera(), x.getIndexer()),
                    new Player(world, false),
                    false);
        }
        player.setState(2, 4, 0, 0); // initial position

        friend1 = new PilotAssembly(
                x -> new SpeakerCycler(x.getDrive(), x.getCamera(), x.getIndexer()),
                new Friend("blue 1", world, false),
                false);
        friend1.setState(1, 1, 0, 0); // initial position

        friend2 = new PilotAssembly(
                x -> new Defender(),
                new Friend("blue 2", world, false),
                false);
        friend2.setState(1, 4, 0, 0); // initial position

        source = new Source(world, new Translation2d(15.5, 1.0));
        source.setDefaultCommand(new SourceDefault(source, world, true, false));
    }

    @Override
    public void reset() {
        player.reset();
        friend1.reset();
        friend2.reset();
    }

    @Override
    public void begin() {
        player.begin();
        friend1.begin();
        friend2.begin();
    }

    @Override
    public void periodic() {
        player.periodic();
        friend1.periodic();
        friend2.periodic();
    }
}
