package org.team100.lib.experiments;

import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.team100.lib.config.Identity;
import org.team100.lib.telemetry.NamedChooser;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

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
    private final Telemetry t = Telemetry.get();

    /** These experiments are enabled on every robot type. */
    private final Set<Experiment> globalExperiments = Set.of(
            Experiment.UseClosedLoopDrive,
            Experiment.UseClosedLoopSteering,
            Experiment.UseClosedLoopVelocity,
            Experiment.UseSetpointGenerator,
            Experiment.UseInitialVelocity,
            Experiment.OscillateDirect);

    /** These experiments are enabled on specific robot types. */
    private final Map<Identity, Set<Experiment>> experimentsByIdentity = Map.of(
            Identity.COMP_BOT, Set.of(
                    Experiment.UseClosedLoopDrive,
                    Experiment.UseClosedLoopSteering,
                    Experiment.UseSetpointGenerator));

    /** Computed for the actual identity used. */
    private final Set<Experiment> m_experiments;

    /** Starts with the config above, but can be overridden. */
    private final Map<Experiment, SendableChooser<BooleanSupplier>> m_overrides;

    public Experiments(Identity identity) {
        m_experiments = EnumSet.copyOf(globalExperiments);
        m_experiments.addAll(experimentsByIdentity.getOrDefault(identity, EnumSet.noneOf(Experiment.class)));
        log();
        m_overrides = new EnumMap<>(Experiment.class);
        for (Experiment e : Experiment.values()) {
            SendableChooser<BooleanSupplier> override = new NamedChooser<>(e.name());
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

    public boolean enabled(Experiment experiment) {
        log();
        return m_overrides.get(experiment).getSelected().getAsBoolean();
        // return m_experiments.contains(experiment);
    }

    ////////////////////////////////////////

    private String on(Experiment e) {
        return e.name() + " ON";
    }

    private String off(Experiment e) {
        return e.name() + " OFF";
    }

    private void log() {
        t.log(Level.INFO, "/experiments/enabled", m_experiments.stream().map(Experiment::name).toArray(String[]::new));
    }

}
