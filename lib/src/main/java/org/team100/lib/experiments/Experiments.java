package org.team100.lib.experiments;

import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.telemetry.ExperimentChooser;

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
public class Experiments implements Glassy {
    public static final Experiments instance = new Experiments(Identity.instance);

    /** These experiments are enabled on every robot type. */
    private final Set<Experiment> globalExperiments = Set.of(
            Experiment.UseSetpointGenerator,
            Experiment.UseInitialVelocity,
            Experiment.OscillateDirect,
            Experiment.HeedVision);

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
        m_experiments = EnumSet.copyOf(globalExperiments);
        m_experiments.addAll(experimentsByIdentity.getOrDefault(identity, EnumSet.noneOf(Experiment.class)));
        m_overrides = new EnumMap<>(Experiment.class);
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
        return m_overrides.get(experiment).getSelected().getAsBoolean();
    }

    

    ////////////////////////////////////////

    @Override
    public String getGlassName() {
        return "Experiments";
    }

    private String on(Experiment e) {
        return e.name() + " ON";
    }

    private String off(Experiment e) {
        return e.name() + " OFF";
    }
}
