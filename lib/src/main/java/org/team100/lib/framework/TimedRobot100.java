package org.team100.lib.framework;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.NotifierJNI;

import java.util.Map;
import java.util.PriorityQueue;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

/**
 * Copy of {@link edu.wpi.first.wpilibj.TimedRobot} in an effort to improve
 * instrumentation.
 */
public class TimedRobot100 extends IterativeRobotBase {

    static class Callback implements Comparable<Callback> {
        public Runnable func;
        public double period;
        public double expirationTime;
        public String name;

        /**
         * Construct a callback container.
         *
         * @param func             The callback to run.
         * @param startTimeSeconds The common starting point for all callback scheduling
         *                         in seconds.
         * @param periodSeconds    The period at which to run the callback in seconds.
         * @param offsetSeconds    The offset from the common starting time in seconds.
         * @param name             for logging
         */
        Callback(Runnable func, double startTimeSeconds, double periodSeconds, double offsetSeconds, String name) {
            this.func = func;
            this.period = periodSeconds;
            this.expirationTime = startTimeSeconds
                    + offsetSeconds
                    + Math.floor((Timer.getFPGATimestamp() - startTimeSeconds) / this.period)
                            * this.period
                    + this.period;
            this.name = name;
        }

        @Override
        public boolean equals(Object rhs) {
            if (rhs instanceof Callback) {
                return Double.compare(expirationTime, ((Callback) rhs).expirationTime) == 0;
            }
            return false;
        }

        @Override
        public int hashCode() {
            return Double.hashCode(expirationTime);
        }

        @Override
        public int compareTo(Callback rhs) {
            // Elements with sooner expiration times are sorted as lesser. The head of
            // Java's PriorityQueue is the least element.
            return Double.compare(expirationTime, rhs.expirationTime);
        }
    }

    /** Default loop period. */
    public static final double kDefaultPeriod = 0.02;

    protected final SupplierLogger2 m_logger;

    // The C pointer to the notifier object. We don't use it directly, it is
    // just passed to the JNI bindings.
    private final int m_notifier = NotifierJNI.initializeNotifier();

    private double m_startTime;

    private final PriorityQueue<Callback> m_callbacks = new PriorityQueue<>();

    private final DoubleSupplierLogger2 m_log_slack;

    /** Constructor for TimedRobot. */
    protected TimedRobot100() {
        this(kDefaultPeriod);
    }

    /**
     * Constructor for TimedRobot.
     *
     * @param period Period in seconds.
     */
    protected TimedRobot100(double period) {
        super(period);
        m_logger = Telemetry.get().namedRootLogger("ROBOT");
        m_log_slack = m_logger.doubleLogger(Level.COMP, "slack time (s)");
        m_startTime = Timer.getFPGATimestamp();
        addPeriodic(this::loopFunc, period, "main loop");
        NotifierJNI.setNotifierName(m_notifier, "TimedRobot");
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Timed);
    }

    @Override
    public void close() {
        NotifierJNI.stopNotifier(m_notifier);
        NotifierJNI.cleanNotifier(m_notifier);
    }

    /** Provide an alternate "main loop" via startCompetition(). */
    @Override
    public void startCompetition() {
        robotInit();

        if (isSimulation()) {
            simulationInit();
        }

        // Tell the DS that the robot is ready to be enabled
        System.out.println("********** Robot program startup complete **********");
        DriverStationJNI.observeUserProgramStarting();

        // Loop forever, calling the appropriate mode-dependent function
        while (true) {
            // We don't have to check there's an element in the queue first because
            // there's always at least one (the constructor adds one). It's reenqueued
            // at the end of the loop.
            var callback = m_callbacks.poll();

            NotifierJNI.updateNotifierAlarm(m_notifier, (long) (callback.expirationTime * 1e6));

            // how long do we spend waiting?
            double startWaitingS = Timer.getFPGATimestamp();
            long curTime = NotifierJNI.waitForNotifierAlarm(m_notifier);
            if (curTime == 0) {
                // someone called StopNotifier
                break;
            }
            double endWaitingS = Timer.getFPGATimestamp();
            double slackS = endWaitingS - startWaitingS;
            // this is the main loop slack, don't let it go to zero!
            m_log_slack.log(() -> slackS);

            log(callback.func::run, callback.name);

            callback.expirationTime += callback.period;
            m_callbacks.add(callback);

            // Process all other callbacks that are ready to run
            // note when we're falling behind, we stay in this inner loop,
            // perhaps never touching the outer loop.
            while ((long) (m_callbacks.peek().expirationTime * 1e6) <= curTime) {
                callback = m_callbacks.poll();

                log(callback.func::run, callback.name);

                callback.expirationTime += callback.period;
                m_callbacks.add(callback);
            }
        }
    }

    private void log(Runnable r, String name) {
        double startWaitingS = Timer.getFPGATimestamp();
        r.run();
        double endWaitingS = Timer.getFPGATimestamp();
        double durationS = endWaitingS - startWaitingS;
        m_logger.doubleLogger(Level.COMP, "duration (s)/" + name).log(() -> durationS);
    }

    /** Ends the main loop in startCompetition(). */
    @Override
    public void endCompetition() {
        NotifierJNI.stopNotifier(m_notifier);
    }

    /**
     * Add a callback to run at a specific period.
     *
     * <p>
     * This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback
     * run
     * synchronously. Interactions between them are thread-safe.
     *
     * @param callback      The callback to run.
     * @param periodSeconds The period at which to run the callback in seconds.
     */
    public final void addPeriodic(Runnable callback, double periodSeconds, String name) {
        m_callbacks.add(new Callback(callback, m_startTime, periodSeconds, 0.0, name));
    }

    /**
     * Add a callback to run at a specific period with a starting time offset.
     *
     * <p>
     * This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback
     * run
     * synchronously. Interactions between them are thread-safe.
     *
     * @param callback      The callback to run.
     * @param periodSeconds The period at which to run the callback in seconds.
     * @param offsetSeconds The offset from the common starting time in seconds.
     *                      This is useful for
     *                      scheduling a callback in a different timeslot relative
     *                      to TimedRobot.
     */
    public final void addPeriodic(Runnable callback, double periodSeconds, double offsetSeconds, String name) {
        m_callbacks.add(new Callback(callback, m_startTime, periodSeconds, offsetSeconds, name));
    }

}
