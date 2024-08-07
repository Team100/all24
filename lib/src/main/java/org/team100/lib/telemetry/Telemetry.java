package org.team100.lib.telemetry;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Simple logging wrapper.
 * 
 * Use keys of the form "/foo/bar"; the slashes separate levels in the tree.
 * 
 * Don't use a slash for any other reason, e.g. for "meters per second" don't
 * say "m/s" say "m_s"
 * 
 * Logged items are "retained" which means they persist even after the logging
 * stops; this means you should see the latest values after disabling the robot.
 */
public class Telemetry {
    public enum Level {
        /**
         * Minimal, curated set of measurements for competition matches. Comp mode
         * overrides root logger enablement.
         */
        COMP(1),
        /**
         * Keep this set small enough to avoid overrunning. This should only contain
         * things you're actively working on.
         */
        DEBUG(2),
        /**
         * Slow, shows absolutely everything. Overruns. Don't expect normal robot
         * behavior in this mode.
         */
        TRACE(3);

        private int priority;

        private Level(int priority) {
            this.priority = priority;
        }

        public boolean admit(Level other) {
            return this.priority >= other.priority;
        }
    }

    /**
     * Logging is prevented after this much time per cycle.
     * TODO: tune this value to leave time for real work.
     */
    private static final double kLoggingTimeBudgetS = 0.01;
    private static final Telemetry instance = new Telemetry();

    private final LoadShedder m_loadShedder;

    final NetworkTableInstance inst;
    final PrimitiveLogger ntLogger;
    final PrimitiveLogger usbLogger;

    private Level m_level;

    /**
     * Uses the default network table instance.
     * Clients should use the static instance, not the constructor.
     */
    private Telemetry() {
        inst = NetworkTableInstance.getDefault();
        m_loadShedder = new LoadShedder(kLoggingTimeBudgetS);
        ntLogger = new NTLogger();
        usbLogger = new DataLogLogger();
        // this will be overridden by {@link TelemetryLevelPoller}
        m_level = Level.TRACE;
        // DataLogManager.start();
        // turn off the CTRE log we never use
        SignalLogger.enableAutoLogging(false);
    }

    public LoadShedder getLoadShedder() {
        return m_loadShedder;
    }

    void setLevel(Level level) {
        m_level = level;
    }

    public Level getLevel() {
        return m_level;
    }

    public static Telemetry get() {
        return instance;
    }

    public FieldLogger fieldLogger(boolean defaultEnabledNT, boolean defaultEnabledUSB) {
        return new FieldLogger(this, defaultEnabledNT, defaultEnabledUSB);
    }

    public RootLogger namedRootLogger(
            String str,
            boolean defaultEnabledNT,
            boolean defaultEnabledUSB) {
        return new RootLogger(this, str, defaultEnabledNT, defaultEnabledUSB);
    }

}