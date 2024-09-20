package frc.robot;

import static java.util.concurrent.TimeUnit.SECONDS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

import edu.wpi.first.networktables.DoublePublisher;

public class Robot extends TimedRobot {
    // the UDP thing can go at 30M key/s which at 50hz is 600000 keys.
    //
    // a not-connected NT client seems to be able to do 7M key/s, so
    // it is overrunning by 400% to do that.
    //
    // a more reasonable value of 60000 keys seems to work disconnected without
    // overrunning.
    //
    // connecting to glass as a local server destroys it, i think because it's
    // trying to send every single update to the GUI?
    //
    // Setting up a separate server (see studies/remote_nt_server) yields
    // lots of connect/disconnect errors at 60k keys; the glass client
    // pointing at the same server never hears anything, and it updates
    // very slowly anyway.
    //
    // 5000 keys seems to work fine; glass consumes 30% of 1 core at that level,
    // which is more than the sender (17%) or receiver (22%).
    private static final int KEYS = 5000;

    private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);

    private final AtomicInteger counter = new AtomicInteger(0);
    // total time spent, note this doesn't count NT threads which are most of the work.
    private final AtomicLong timer = new AtomicLong();

    NetworkTableInstance inst;
    List<DoublePublisher> publishers = new ArrayList<>();

    public Robot() {
        scheduler.scheduleAtFixedRate(
                () -> {
                    System.out.printf("counter %d timer us %d\n",
                            counter.getAndSet(0),
                            timer.getAndSet(0));
                },
                0, 1, SECONDS);
        NetworkTableInstance.getDefault().stopServer();
        // not the default instance!
        inst = NetworkTableInstance.create();
        // inst.setServer("10.1.0.16");
        inst.setServer("localhost");
        inst.startClient4("rio_nt_client");
        for (int i = 0; i < KEYS; ++i) {
            var t = inst.getDoubleTopic("double" + i);
            var p = t.publish(PubSubOption.keepDuplicates(true));
            t.setRetained(true);
            publishers.add(p);
        }
    }

    @Override
    public void robotInit() {
    }

    @Override
    public void robotPeriodic() {
        // double timeSec = Timer.getFPGATimestamp();
        // int flushcounter = 0;
        // for (DoublePublisher p : publishers) {
        // counter.incrementAndGet();
        // p.set(timeSec);
        // if (flushcounter++ > 100) {
        // inst.flush();
        // flushcounter = 0;
        // }
        // }
    }

    @Override
    protected void loopFunc() {
        double timeSec = Timer.getFPGATimestamp();
        var t0_us = RobotController.getFPGATime();
        int flushcounter = 0;
        for (DoublePublisher p : publishers) {
            counter.incrementAndGet();
            p.set(timeSec);
            if (flushcounter++ > 100) {
                inst.flush();
                flushcounter = 0;
            }
        }
        var t1_us = RobotController.getFPGATime();
        timer.addAndGet(t1_us - t0_us);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

}
