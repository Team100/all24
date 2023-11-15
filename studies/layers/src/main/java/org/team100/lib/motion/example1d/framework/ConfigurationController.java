package org.team100.lib.motion.example1d.framework;

public interface ConfigurationController<T extends Configuration<T>, U extends Actuation<U>> {
    U calculate(T measurement, T setpoint);
}
