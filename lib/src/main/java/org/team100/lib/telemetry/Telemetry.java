package org.team100.lib.telemetry;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.team100.lib.dashboard.Glassy;

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
        /** Log nothing for maximum speed */
        SILENT(1),
        /** Serious problems */
        ERROR(2),
        /** Not-serious problems */
        WARN(3),
        /** useful for normal operations */
        INFO(4),
        /** primarily useful during development */
        DEBUG(5),
        /** slow, shows absolutely everything */
        TRACE(6);

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
    static final String kName = "Telemetry";

    final Chronos m_chronos = Chronos.get();
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

    public static Telemetry get() {
        return instance;
    }

    public Logger rootLogger(Glassy obj) {
        return new RootLogger(this, obj.getGlassName());
    }

    public Logger fieldLogger() {
        return new RootLogger(this, "field");
    }

    public Logger namedRootLogger(String str) {
        return new RootLogger(this, str);
    }
}