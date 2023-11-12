package org.team100.lib.motion.example1d.framework;

public interface ConfigurationController<Conf, Act> {
    Actuation<Act> calculate(Configuration<Conf> config);
}
