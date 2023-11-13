package org.team100.persistent_parameter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Preferences;

/**
 * A parameter useful for tuning.
 * 
 * Can be influenced five ways:
 * 
 * 1. load the persistent value from Preference storage
 * 2. default in code at startup if no persistent value
 * 3. update via Network Tables (e.g. glass or coprocessor)
 * 4. update via HID axes, e.g. the custom arduino knobs Joel made.
 * 5. reset to the default via a HID button
 * 
 * Updates are sent to preference storage and the network.
 */
public class PersistentParameter implements DoubleSupplier {
    private final String m_key;
    private final double m_defaultValue;
    private final DoubleSupplier m_knob;
    private double m_knob_offset;

    /** Zero default. */
    public PersistentParameter(String key) {
        this(key, 0.0);
    }

    /** No knob input. */
    public PersistentParameter(String key, double defaultValue) {
        this(key, defaultValue, () -> 0.0);
    }

    /**
     * Use the persisted value if it exists.
     * 
     * @param key          for network tables and preference json file
     * @param defaultValue if no persisted value exists
     * @param knob         update from HID
     */
    public PersistentParameter(String key, double defaultValue, DoubleSupplier knob) {
        m_key = key;
        m_defaultValue = defaultValue;
        m_knob = knob;
        m_knob_offset = knob.getAsDouble();
        Preferences.initDouble(key, defaultValue);
    }

    /** Set the value in the store and the network. */
    public synchronized void set(double value) {
        Preferences.setDouble(m_key, value);
    }

    /**
     * Get the value most recently received (or read from storage), adjusted by the
     * knob.
     */
    public synchronized double get() {
        double pVal = Preferences.getDouble(m_key, m_defaultValue);
        double knobVal = m_knob.getAsDouble();
        double knobNet = knobVal - m_knob_offset;
        m_knob_offset = knobVal;
        double newVal = pVal + knobNet;
        Preferences.setDouble(m_key, newVal);
        return newVal;
    }

    /**
     * Set the store and the network to the default value.
     * This is intended for onTrue(runOnce()).
     */
    public synchronized void reset() {
        System.out.println("reset");
        m_knob_offset = m_knob.getAsDouble();
        Preferences.setDouble(m_key, m_defaultValue);
    }

	@Override
	public double getAsDouble() {
		return get();
	}
}
