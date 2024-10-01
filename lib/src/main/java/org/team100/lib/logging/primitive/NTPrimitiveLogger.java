package org.team100.lib.logging.primitive;

import java.util.HashSet;
import java.util.Set;
import java.util.stream.Stream;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * This is a sort of back-up option for logging directly to a local network
 * tables server.
 * 
 * Logged items are "retained" which means they persist even after the logging
 * stops; this means you should see the latest values after disabling the robot.
 * 
 * With a full load of logging this will overrun the 50hz loop very badly, so if
 * you use it, you'll need to turn off most of the logging.
 */
public class NTPrimitiveLogger implements PrimitiveLogger {
    private final NetworkTableInstance inst;
    // this is duplicative of the NT topic list, but the NT topics includes
    // other non-logging keys, so we keep our own list.
    private final Set<String> keys = new HashSet<>();

    public NTPrimitiveLogger() {
        inst = NetworkTableInstance.getDefault();
        // Also log to disk
        DataLogManager.start();
    }

    @Override
    public int keyCount() {
        return keys.size();
    }

    public class NTBooleanLogger implements PrimitiveLogger.PrimitiveBooleanLogger {
        BooleanPublisher m_pub;

        public NTBooleanLogger(String label) {
            BooleanTopic t = inst.getBooleanTopic(label);
            m_pub = t.publish();
            t.setRetained(true);
        }

        @Override
        public void log(boolean val) {
            m_pub.set(val);
        }

    }

    public class NTDoubleLogger implements PrimitiveLogger.PrimitiveDoubleLogger {
        DoublePublisher m_pub;

        public NTDoubleLogger(String label) {
            DoubleTopic t = inst.getDoubleTopic(label);
            m_pub = t.publish();
            t.setRetained(true);
        }

        @Override
        public void log(double val) {
            m_pub.set(val);
        }
    }

    public class NTIntLogger implements PrimitiveLogger.PrimitiveIntLogger {
        IntegerPublisher m_pub;

        public NTIntLogger(String label) {
            IntegerTopic t = inst.getIntegerTopic(label);
            m_pub = t.publish();
            t.setRetained(true);
        }

        @Override
        public void log(int val) {
            m_pub.set(val);
        }
    }

    public class NTDoubleArrayLogger implements PrimitiveLogger.PrimitiveDoubleArrayLogger {
        DoubleArrayPublisher m_pub;

        public NTDoubleArrayLogger(String label) {
            DoubleArrayTopic t = inst.getDoubleArrayTopic(label);
            m_pub = t.publish();
            t.setRetained(true);
        }

        @Override
        public void log(double[] val) {
            m_pub.set(val);
        }
    }

    public class NTLongLogger implements PrimitiveLogger.PrimitiveLongLogger {
        IntegerPublisher m_pub;

        public NTLongLogger(String label) {
            IntegerTopic t = inst.getIntegerTopic(label);
            m_pub = t.publish();
            t.setRetained(true);
        }

        @Override
        public void log(long val) {
            m_pub.set(val);
        }
    }

    public class NTStringLogger implements PrimitiveLogger.PrimitiveStringLogger {
        StringPublisher m_pub;

        public NTStringLogger(String label) {
            StringTopic t = inst.getStringTopic(label);
            m_pub = t.publish();
            t.setRetained(true);
        }

        @Override
        public void log(String val) {
            m_pub.set(val);
        }
    }

    @Override
    public PrimitiveBooleanLogger booleanLogger(String label) {
        keys.add(label);
        return new NTBooleanLogger(label);
    }

    @Override
    public PrimitiveDoubleLogger doubleLogger(String label) {
        keys.add(label);
        return new NTDoubleLogger(label);
    }

    @Override
    public PrimitiveIntLogger intLogger(String label) {
        keys.add(label);
        return new NTIntLogger(label);
    }

    @Override
    public PrimitiveDoubleArrayLogger doubleArrayLogger(String label) {
        keys.add(label);
        return new NTDoubleArrayLogger(label);
    }

    @Override
    public PrimitiveLongLogger longLogger(String label) {
        keys.add(label);
        return new NTLongLogger(label);
    }

    @Override
    public PrimitiveStringLogger stringLogger(String label) {
        keys.add(label);
        return new NTStringLogger(label);
    }

}
