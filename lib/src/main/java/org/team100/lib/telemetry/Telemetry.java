package org.team100.lib.telemetry;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.DataLogManager;

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
     * useful for troubleshooting unit tests. it's quite slow.
     */
    static final boolean kAlsoPrint = false;
    private static final Telemetry instance = new Telemetry();

    final NetworkTableInstance inst;
    final Map<String, Publisher> pubs;
    Level m_level;

    /**
     * Uses the default network table instance.
     * Clients should use the static instance, not the constructor.
     */
    private Telemetry() {
        inst = NetworkTableInstance.getDefault();
        pubs = new ConcurrentHashMap<>();
        // this will be overridden by {@link TelemetryLevelPoller}
        m_level = Level.TRACE;
        DataLogManager.start();
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

    public FieldLogger fieldLogger(boolean defaultEnabled) {
        return new FieldLogger(this, defaultEnabled);
    }

    public RootLogger namedRootLogger(String str, boolean defaultEnabled) {
        return new RootLogger(this, str, defaultEnabled);
    }
}