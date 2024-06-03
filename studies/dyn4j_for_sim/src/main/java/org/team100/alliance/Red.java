package org.team100.alliance;

import org.team100.commands.SourceDefault;
import org.team100.control.Idlepilot;
import org.team100.control.SelectorPilot;
import org.team100.control.auto.Defender;
import org.team100.control.auto.Passer;
import org.team100.control.auto.Scorer;
import org.team100.robot.RobotAssembly;
import org.team100.robot.Source;
import org.team100.sim.Foe;
import org.team100.sim.SimWorld;

import edu.wpi.first.math.geometry.Translation2d;

public class Red implements Alliance {
    private final RobotAssembly scorer;
    private final RobotAssembly passer;
    private final RobotAssembly defender;
    private final Source source;

    public Red(SimWorld world) {
        scorer = new RobotAssembly(
                x -> SelectorPilot.autonSelector(
                        new Idlepilot(),
                        new Scorer(x.getDrive(), x.getCamera(), x.getIndexer())),
                new Foe("red scorer", world, false),
                false);
        scorer.setState(15, 3, 0, 0);

        passer = new RobotAssembly(
                x -> SelectorPilot.autonSelector(
                        new Idlepilot(),
                        new Passer(x.getDrive(), x.getCamera(), x.getIndexer())),
                new Foe("red passer", world, false),
                false);
        passer.setState(15, 5, 0, 0);

        defender = new RobotAssembly(
                x -> SelectorPilot.autonSelector(
                        new Idlepilot(),
                        new Defender()),
                new Foe("red defender", world, false),
                false);
        defender.setState(13, 7, 0, 0);

        source = new Source(world, new Translation2d(1.0, 1.0));
        source.setDefaultCommand(new SourceDefault(source, world, false, false));
    }

    @Override
    public void reset() {
        scorer.reset();
        passer.reset();
        defender.reset();
    }

    @Override
    public void begin() {
        scorer.begin();
        passer.begin();
        defender.begin();
    }

    @Override
    public void periodic() {
        scorer.periodic();
        passer.periodic();
        defender.periodic();
    }
}
