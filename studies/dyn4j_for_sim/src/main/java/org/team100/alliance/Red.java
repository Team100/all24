package org.team100.alliance;

import org.team100.commands.SourceDefault;
import org.team100.control.Pilot;
import org.team100.control.auto.Defender;
import org.team100.control.auto.Passer;
import org.team100.control.auto.Scorer;
import org.team100.robot.PilotAssembly;
import org.team100.robot.RobotAssembly;
import org.team100.robot.Source;
import org.team100.sim.Foe;
import org.team100.sim.SimWorld;

import edu.wpi.first.math.geometry.Translation2d;

public class Red implements Alliance {
    private static final Translation2d kSpeaker = new Translation2d(16.541, 5.548);
    private final RobotAssembly scorer;
    private final RobotAssembly passer;
    private final RobotAssembly defender;
    private final Source source;
    private final Pilot scoreAlternator;
    private final Pilot passCycler;
    private final Pilot defenseOnly;

    public Red(SimWorld world) {
        Foe scorerBody = new Foe("red scorer", world);
        scoreAlternator = new Scorer();
        scorer = new PilotAssembly(scoreAlternator, scorerBody, kSpeaker);
        scorer.setState(15, 3, 0, 0);
        world.addBody(scorerBody);

        Foe red2 = new Foe("red 2", world);
        passCycler = new Passer();
        passer = new PilotAssembly(passCycler, red2, kSpeaker);
        passer.setState(15, 5, 0, 0);
        world.addBody(red2);

        Foe red3 = new Foe("red 3", world);
        defenseOnly = new Defender();
        defender = new PilotAssembly(defenseOnly, red3, kSpeaker);
        defender.setState(13, 7, 0, 0);
        world.addBody(red3);

        source = new Source(world, 1.0, 1.0);
        source.setDefaultCommand(new SourceDefault(source));
    }

    @Override
    public void reset() {
        scoreAlternator.reset();
        passCycler.reset();
        defenseOnly.reset();
        scorer.setState(15, 3, 0, 0);
        passer.setState(15, 5, 0, 0);
        defender.setState(13, 7, 0, 0);
    }

    @Override
    public void begin() {
        scoreAlternator.begin();
        passCycler.begin();
        defenseOnly.begin();
    }

    @Override
    public void periodic() {
        scoreAlternator.periodic();
        passCycler.periodic();
        defenseOnly.periodic();
    }
}
