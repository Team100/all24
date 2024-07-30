package org.team100.lib.sensors;

import java.util.EnumSet;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.ValueEventData;

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
    /** For now, we just average all the inputs. */
    private static final double kAuthority = 0.5;

    private final NetworkTableListenerPoller m_poller;

    private double m_yaw;
    private double m_yawRate;
    private double m_pitch;
    private double m_roll;

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
        return new Rotation2d(m_yaw);
    }

    @Override
    public double getYawRateNWU() {
        update();
        return m_yawRate;
    }

    @Override
    public Rotation2d getPitchNWU() {
        update();
        return new Rotation2d(m_pitch);
    }

    @Override
    public Rotation2d getRollNWU() {
        update();
        return new Rotation2d(m_roll);
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
            // TODO: do something with timestamp, maybe include it in the gyro API
            long timestamp = v.getServerTime();
            String measure = fields[2];
            if (measure.equals(kYaw)) {
                m_yaw = updateMeasure(m_yaw, v);
            } else if (measure.equals(kYawRate)) {
                m_yawRate = updateMeasure(m_yawRate, v);
            }
            // TODO: add the other dimensions
        }
    }

    private double updateMeasure(double val, NetworkTableValue v) {
        return (1 - kAuthority) * val + kAuthority * v.getDouble();
    }
}
