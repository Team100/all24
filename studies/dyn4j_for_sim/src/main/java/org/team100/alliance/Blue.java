package org.team100.alliance;

import org.team100.commands.SourceDefault;
import org.team100.control.ManualPilot;
import org.team100.control.Pilot;
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
import edu.wpi.first.wpilibj2.command.Command;

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
    private final Pilot ampCycler;
    private final Pilot speakerCycler;
    private final Pilot defender;

    public Blue(SimWorld world) {
        Player playerBody = new Player(world);
        // might not be used below.
        ampCycler = new AmpCycler();
        if (kRealPlayer) {
            player = new RealPlayerAssembly(new ManualPilot(), playerBody, kSpeaker);
            // use the pilot assembly with manual control, to test the buttons.
            // player = new PilotAssembly(new ManualPilot(), playerBody, kSpeaker);
        } else {
            player = new PilotAssembly(ampCycler, playerBody, kSpeaker);
        }
        player.setState(2, 4, 0, 0); // initial position
        world.addBody(playerBody);

        Friend blue1 = new Friend("blue 1", world);
        speakerCycler = new SpeakerCycler();
        friend1 = new PilotAssembly(speakerCycler, blue1, kSpeaker);
        friend1.setState(1, 1, 0, 0); // initial position
        world.addBody(blue1);

        Friend blue2 = new Friend("blue 2", world);
        defender = new Defender();
        friend2 = new PilotAssembly(defender, blue2, kSpeaker);
        friend2.setState(1, 4, 0, 0); // initial position
        world.addBody(blue2);

        source = new Source(world, 15.5, 1.0);
        source.setDefaultCommand(new SourceDefault(source));
    }

    /**
     * Set up locations.
     * State machine to initial.
     */
    public void reset() {
        ampCycler.reset();
        speakerCycler.reset();
        defender.reset();
        // initial position
        player.setState(2, 4, 0, 0);
        friend1.setState(1, 1, 0, 0);
        friend2.setState(1, 4, 0, 0);
    }

    /** State machine to first active state */
    public void begin() {
        ampCycler.begin();
        speakerCycler.begin();
        defender.begin();
    }

    public void periodic() {
        ampCycler.periodic();
        speakerCycler.periodic();
        defender.periodic();
    }

    @Override
    public void onEnd(RobotAssembly robot, Command command) {
        //
    }

}
