package org.team100.lib.experiments;

import java.util.EnumSet;
import java.util.Map;
import java.util.Set;

import org.team100.lib.config.Identity;
import org.team100.lib.telemetry.Telemetry;

public class Experiments {
    private final Telemetry t = Telemetry.get();

    /** These experiments are enabled on every robot type. */
    private final Set<Experiment> globalExperiments = Set.of(
            Experiment.UseClosedLoopDrive,
            Experiment.UseClosedLoopSteering,
            Experiment.UseSetpointGenerator);

    /** These experiments are enabled on specific robot types. */
    private final Map<Identity, Set<Experiment>> experimentsByIdentity = Map.of(
            Identity.COMP_BOT, Set.of(
                    Experiment.UseClosedLoopDrive,
                    Experiment.UseClosedLoopSteering,
                    Experiment.UseSetpointGenerator));

    /** Computed for the actual identity used. */
    private final Set<Experiment> m_experiments;

    public Experiments(Identity identity) {
        m_experiments = EnumSet.copyOf(globalExperiments);
        m_experiments.addAll(experimentsByIdentity.getOrDefault(identity, EnumSet.noneOf(Experiment.class)));
        t.log("/experiments/enabled", m_experiments.stream().map(Experiment::name).toArray(String[]::new));
    }

    public boolean enabled(Experiment experiment) {
        return m_experiments.contains(experiment);
    }
}
