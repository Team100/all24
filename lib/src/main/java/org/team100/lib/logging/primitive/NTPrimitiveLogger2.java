package org.team100.lib.logging.primitive;

import java.util.HashSet;
import java.util.Set;
import java.util.stream.Stream;

import org.team100.lib.logging.primitive.PrimitiveLogger2.BooleanLogger;
import org.team100.lib.logging.primitive.PrimitiveLogger2.DoubleArrayLogger;
import org.team100.lib.logging.primitive.PrimitiveLogger2.DoubleLogger;
import org.team100.lib.logging.primitive.PrimitiveLogger2.DoubleObjArrayLogger;
import org.team100.lib.logging.primitive.PrimitiveLogger2.IntLogger;
import org.team100.lib.logging.primitive.PrimitiveLogger2.LongLogger;
import org.team100.lib.logging.primitive.PrimitiveLogger2.StringLogger;

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
public class NTPrimitiveLogger2 implements PrimitiveLogger2 {
    private final NetworkTableInstance inst;
    // this is duplicative of the NT topic list, but the NT topics includes
    // other non-logging keys, so we keep our own list.
    private final Set<String> keys = new HashSet<>();

    public NTPrimitiveLogger2() {
        inst = NetworkTableInstance.getDefault();
        // Also log to disk
        DataLogManager.start();
    }

    @Override
    public int keyCount() {
        return keys.size();
    }

    public class NTBooleanLogger implements PrimitiveLogger2.BooleanLogger {
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

    public class NTDoubleLogger implements PrimitiveLogger2.DoubleLogger {
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

    public class NTIntLogger implements PrimitiveLogger2.IntLogger {
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

    public class NTDoubleArrayLogger implements PrimitiveLogger2.DoubleArrayLogger {
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

    public class NTDoubleObjArrayLogger implements PrimitiveLogger2.DoubleObjArrayLogger {
        DoubleArrayPublisher m_pub;

        public NTDoubleObjArrayLogger(String label) {
            DoubleArrayTopic t = inst.getDoubleArrayTopic(label);
            m_pub = t.publish();
            t.setRetained(true);
        }

        @Override
        public void log(Double[] val) {
            m_pub.set(Stream.of(val).mapToDouble(Double::doubleValue).toArray());
        }
    }

    public class NTLongLogger implements PrimitiveLogger2.LongLogger {
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

    public class NTStringLogger implements PrimitiveLogger2.StringLogger {
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
    public BooleanLogger booleanLogger(String label) {
        keys.add(label);
        return new NTBooleanLogger(label);
    }

    @Override
    public DoubleLogger doubleLogger(String label) {
        keys.add(label);
        return new NTDoubleLogger(label);
    }

    @Override
    public IntLogger intLogger(String label) {
        keys.add(label);
        return new NTIntLogger(label);
    }

    @Override
    public DoubleArrayLogger doubleArrayLogger(String label) {
        keys.add(label);
        return new NTDoubleArrayLogger(label);
    }

    @Override
    public DoubleObjArrayLogger doubleObjArrayLogger(String label) {
        keys.add(label);
        return new NTDoubleObjArrayLogger(label);
    }

    @Override
    public LongLogger longLogger(String label) {
        keys.add(label);
        return new NTLongLogger(label);
    }

    @Override
    public StringLogger stringLogger(String label) {
        keys.add(label);
        return new NTStringLogger(label);
    }

}
