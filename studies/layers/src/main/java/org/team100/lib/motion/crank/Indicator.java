package org.team100.lib.motion.crank;

import java.util.function.Supplier;

import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Use the scheduler button loop to update console indicators. */
public class Indicator {
    public interface Visible {
        void accept(Indicator indicator);
    }

    private final Telemetry t = new Telemetry();
    private final EventLoop m_loop;
    private final HID m_hid;
    private final Supplier<Visible> m_root;
    /** Accumulates indicator bits on each iteration. */
    int indicators;

    /** @param root is a lambda so it can change. */
    public Indicator(HID hid, Supplier<Visible> root) {
        m_loop = CommandScheduler.getInstance().getDefaultButtonLoop();
        m_hid = hid;
        m_root = root;
    }

    public void start() {
        m_loop.bind(this::rooter);
    }

    public void rooter() {
        indicators = 0;
        // walk the tree, setting bits as we go.
        m_root.get().accept(this);
        m_hid.genericHID.setOutputs(indicators);
    }

    /** catch-all */
    public void indicate(Visible visible) {
        log(visible);
    }

    private void log(Visible visible) {
        String name = visible.getClass().getSimpleName();
        t.log("/indicator/" + name, true);
    }

    // one-hot indicator encoding, keep this in order.

    public void indicate(CrankSubsystem visible) {
        log(visible);
        indicators |= 0b0000_0000_0000_0001;
    }

    public void indicate(ActuatorOnboard visible) {
        log(visible);
        indicators |= 0b0000_0000_0000_0010;
    }

    public void indicate(ActuatorOutboard visible) {
        log(visible);
        indicators |= 0b0000_0000_0000_0100;
    }

    public void indicate(ActuationFilter visible) {
        log(visible);
        indicators |= 0b0000_0000_0000_1000;
    }

    public void indicate(ActuationZero visible) {
        log(visible);
        indicators |= 0b0000_0000_0001_0000;
    }

    public void indicate(ActuationManual visible) {
        log(visible);
        indicators |= 0b0000_0000_0010_0000;
    }

    public void indicate(ConfigurationController visible) {
        log(visible);
        indicators |= 0b0000_0000_0100_0000;
    }

    public void indicate(ConfigurationZero visible) {
        log(visible);
        indicators |= 0b0000_0000_1000_0000;
    }

}
