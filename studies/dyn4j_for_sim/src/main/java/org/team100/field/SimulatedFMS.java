package org.team100.field;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

/**
 * Uses the simulated driver station to run a match.
 * 
 * Supply a callback to get notified when the match is over (e.g. to show the
 * final score).
 */
public class SimulatedFMS {
    private final ScheduledExecutorService m_scheduler;
    protected final CallbackStore m_terminate;
    private double m_time;
    private boolean m_finished;

    public SimulatedFMS() {
        m_scheduler = Executors.newSingleThreadScheduledExecutor();
        // turn on "test" mode to terminate the match.
        m_terminate = DriverStationSim.registerTestCallback((name, value) -> finish(), false);
        m_time = 0;
        m_finished = false;
    }

    public void start() {
        init();
        at(1, this::countThree);
        at(2, this::countTwo);
        at(3, this::countOne);
        at(4, this::enableAuton);
        at(19, this::disable);
        // 3 second pause here
        at(22, this::enableTeleop);
        at(157, this::disable);
        // 5 second pause here
        at(162, this::finish);
    }

    public boolean isFinished() {
        return m_finished;
    }

    /** Simulated getMatchTime seems not to work, so there's this. */
    public double getMatchTime() {
        if (DriverStationSim.getEnabled()) {
            double elapsed = Timer.getFPGATimestamp() - m_time;
            if (DriverStation.isAutonomous()) {
                return 15 - elapsed;
            } else {
                return 135 - elapsed;
            }
        } else {
            if (DriverStation.isAutonomous()) {
                // enabled and autonomous == in the 3 sec after auton
                return 135;
            } else {
                // enabled and not autonomous = in the 5 sec after teleop
                return 0;
            }
        }
    }

    private void init() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEventName("Simulation");
        DriverStationSim.setFmsAttached(true);
        DriverStationSim.setMatchNumber(1);
        DriverStationSim.setMatchType(MatchType.Practice);
        DriverStationSim.notifyNewData();
    }

    private void at(long sec, Runnable r) {
        m_scheduler.schedule(r, sec, TimeUnit.SECONDS);
    }

    private void countThree() {
        System.out.println("THREE!");
    }

    private void countTwo() {
        System.out.println("TWO!");
    }

    private void countOne() {
        System.out.println("ONE!");
    }

    private void enableAuton() {
        System.out.println("CRESCENDO!");
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        m_time = Timer.getFPGATimestamp();
    }

    private void disable() {
        System.out.println("BUZZER!");
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    private void enableTeleop() {
        System.out.println("DING DING DING!");
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        m_time = Timer.getFPGATimestamp();
    }

    private void finish() {
        System.out.println("FINISHED!");
        m_finished = true;
        DriverStationSim.setFmsAttached(false);
        m_scheduler.shutdownNow();
    }

}
