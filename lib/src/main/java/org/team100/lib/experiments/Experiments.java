package org.team100.lib.experiments;

import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.team100.lib.config.Identity;
import org.team100.lib.telemetry.ExperimentChooser;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controls Experiment enablement.
 * 
 * There are three methods of enablement:
 * 
 * -- global: enabled for all robots
 * -- per-identity: enabled for specific RoboRIO serial numbers
 * -- override: using a Sendable Chooser in a dashboard, e.g. glass.
 */
public class Experiments {
    public static final Experiments instance = new Experiments(Identity.instance);
    private final Telemetry t = Telemetry.get();
    private final String m_name;

    /** These experiments are enabled on every robot type. */
    private final Set<Experiment> globalExperiments = Set.of(
            Experiment.UseSetpointGenerator,
            Experiment.UseInitialVelocity,
            Experiment.OscillateDirect);

    /** These experiments are enabled on specific robot types. */
    private final Map<Identity, Set<Experiment>> experimentsByIdentity = Map.of(
            Identity.COMP_BOT, Set.of(
                    Experiment.UseSetpointGenerator));

    /** Computed for the actual identity used. */
    private final Set<Experiment> m_experiments;

    /** Starts with the config above, but can be overridden. */
    private final Map<Experiment, SendableChooser<BooleanSupplier>> m_overrides;

    private final Map<Experiment, Boolean> m_testOverrides;

    private Experiments(Identity identity) {
        m_name = Names.name(this);
        m_experiments = EnumSet.copyOf(globalExperiments);
        m_experiments.addAll(experimentsByIdentity.getOrDefault(identity, EnumSet.noneOf(Experiment.class)));
        m_overrides = new EnumMap<>(Experiment.class);
        log();
        m_testOverrides = new EnumMap<>(Experiment.class);
        for (Experiment e : Experiment.values()) {
            SendableChooser<BooleanSupplier> override = ExperimentChooser.get(e.name());
            if (m_experiments.contains(e)) {
                override.setDefaultOption(on(e), () -> true);
                override.addOption(off(e), () -> false);
            } else {
                override.addOption(on(e), () -> true);
                override.setDefaultOption(off(e), () -> false);
            }
            m_overrides.put(e, override);
            SmartDashboard.putData(override);
        }
    }

    /** overrides everything. for testing only. */
    public void testOverride(Experiment experiment, boolean state) {
        m_testOverrides.put(experiment, state);
    }

    public boolean enabled(Experiment experiment) {
        if (m_testOverrides.containsKey(experiment)) {
            return m_testOverrides.get(experiment);
        }
        log();
        return m_overrides.get(experiment).getSelected().getAsBoolean();
    }

    ////////////////////////////////////////

    private String on(Experiment e) {
        return e.name() + " ON";
    }

    private String off(Experiment e) {
        return e.name() + " OFF";
    }

    private void log() {
        // the enabled experiments are only logged here for analysis, not control.
        t.log(Level.DEBUG, m_name, "enabled",
                () -> m_overrides.entrySet()
                        .stream()
                        .filter(e -> e.getValue().getSelected().getAsBoolean())
                        .map(Map.Entry::getKey)
                        .map(Experiment::name)
                        .toArray(String[]::new));
    }

}
