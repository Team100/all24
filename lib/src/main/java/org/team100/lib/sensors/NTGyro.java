package org.team100.lib.sensors;

import java.util.EnumSet;

import org.team100.lib.util.TimestampedDouble;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.wpilibj.Timer;

/**
 * Gyro data from network tables.
 * 
 * This is the other half of the python gyro server
 * currently in studies/python_deployment.
 * 
 * TODO: address startup transients
 * TODO: convert to Optional
 * TODO: better fusion than just averaging (e.g. detect bad input)
 * TODO: notice if we have stale data
 */
public class NTGyro implements Gyro {
    private static final String kYaw = "gyro_yaw";
    private static final String kYawRate = "gyro_rate";
    private static final String kPitch = "gyro_pitch";
    private static final String kRoll = "gyro_roll";
    /** For now, we just average all the inputs. */
    private static final double kAuthority = 0.5;

    private final NetworkTableListenerPoller m_poller;

    private TimestampedDouble m_yaw;
    private TimestampedDouble m_yawRate;
    private TimestampedDouble m_pitch;
    private TimestampedDouble m_roll;

    public NTGyro() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_poller = new NetworkTableListenerPoller(inst);
        m_poller.addListener(
                new MultiSubscriber(inst, new String[] { "vision" }),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll));

    }

    @Override
    public Rotation2d getYawNWU() {
        update();
        // extrapolate to now, assuming the rate is current
        double nowS = Timer.getFPGATimestamp();
        double dtS = nowS - m_yaw.getTimeS();
        double dYaw = m_yawRate.getValue() * dtS;
        double nowYaw = m_yaw.getValue() + dYaw;
        return new Rotation2d(nowYaw);
    }

    @Override
    public double getYawRateNWU() {
        update();
        // TODO: extrapolate based on yaw acceleration?
        return m_yawRate.getValue();
    }

    @Override
    public Rotation2d getPitchNWU() {
        update();
        // TODO: extrapolate based on pitch rate
        return new Rotation2d(m_pitch.getValue());
    }

    @Override
    public Rotation2d getRollNWU() {
        update();
        // TODO: extrapolate based on roll rate
        return new Rotation2d(m_roll.getValue());
    }

    ///////////////////////////////////

    /**
     * Polls the subscriber and updates members if required.
     * 
     * Doing this update periodically, even many times unnecessarily, is many orders
     * of magnitude faster than context switching into a separate thread to do the
     * update, because the thread switching is expensive.
     */
    private void update() {

        NetworkTableEvent[] events = m_poller.readQueue();
        for (NetworkTableEvent e : events) {
            ValueEventData ve = e.valueData;
            String name = ve.getTopic().getName();
            String[] fields = name.split("/");
            if (fields.length != 3)
                continue;
            NetworkTableValue v = ve.value;
            if (!v.isValid())
                continue;
            if (!v.isDouble())
                continue;
            String measure = fields[2];
            if (measure.equals(kYaw)) {
                m_yaw = updateMeasure(m_yaw, v);
            } else if (measure.equals(kYawRate)) {
                m_yawRate = updateMeasure(m_yawRate, v);
            } else if (measure.equals(kPitch)) {
                m_pitch = updateMeasure(m_pitch, v);
            } else if (measure.equals(kRoll)) {
                m_roll = updateMeasure(m_roll, v);
            }
        }
    }

    private TimestampedDouble updateMeasure(TimestampedDouble val, NetworkTableValue v) {
        // TODO: if it's been a long time since the previous value, just use the new
        // one.
        return new TimestampedDouble(
                (1 - kAuthority) * val.getValue() + kAuthority * v.getDouble(),
                v.getServerTime() / 1000000.0);
    }
}
