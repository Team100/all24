package org.team100.alliance;

import org.team100.commands.SourceDefault;
import org.team100.control.ManualPilot;
import org.team100.control.SelectorPilot;
import org.team100.control.auto.AmpCycler;
import org.team100.control.auto.Auton;
import org.team100.control.auto.Defender;
import org.team100.control.auto.SpeakerCycler;
import org.team100.robot.RobotAssembly;
import org.team100.robot.Source;
import org.team100.sim.Friend;
import org.team100.sim.Player;
import org.team100.sim.SimWorld;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        // near 3
        if (kRealPlayer) {
            player = new RobotAssembly(
                    x -> SelectorPilot.autonSelector(
                            new Auton(x.getDrive(), x.getCamera(), x.getIndexer(),
                                    new Pose2d(3.0, 5.5, new Rotation2d(Math.PI)),
                                    1, 2, 3),
                            new ManualPilot()),
                    new Player(world, false),
                    false);
            // use the pilot assembly with manual control, to test the buttons.
            // player = new PilotAssembly(x -> new ManualPilot(), playerBody, kSpeaker);
        } else {
            player = new RobotAssembly(
                    x -> SelectorPilot.autonSelector(
                            new Auton(x.getDrive(), x.getCamera(), x.getIndexer(),
                                    new Pose2d(3.0, 7.0, new Rotation2d(Math.PI)),
                                    3, 2, 1),
                            new AmpCycler(x.getDrive(), x.getCamera(), x.getIndexer())),
                    new Player(world, false),
                    false);
        }
        player.setState(2, 7, 0, 0); // initial position

        // far 3
        friend1 = new RobotAssembly(
                x -> SelectorPilot.autonSelector(
                        new Auton(x.getDrive(), x.getCamera(), x.getIndexer(),
                                new Pose2d(3.0, 5.5, new Rotation2d(Math.PI)),
                                8, 7, 6),
                        new SpeakerCycler(x.getDrive(), x.getCamera(), x.getIndexer(),
                                new Pose2d(3.0, 5.5, new Rotation2d(Math.PI)))),
                new Friend("blue 1", world, false),
                false);
        friend1.setState(2, 5.5, 0, 0); // initial position

        // complement 2
        friend2 = new RobotAssembly(
                x -> SelectorPilot.autonSelector(
                        new Auton(x.getDrive(), x.getCamera(), x.getIndexer(),
                                new Pose2d(3.0, 3.0, new Rotation2d(Math.PI)),
                                4, 5),
                        new Defender()),
                new Friend("blue 2", world, false),
                false);
        friend2.setState(2, 3, 0, 0); // initial position

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
