package org.team100.lib.trajectory;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

public class ManualDriveReferenceGenerator implements ReferenceGenerator<Pose2d> {
    public static class Config {
        public double scale;
    }

    public static class ManualSample implements ReferenceGenerator.Sample<Pose2d> {
        private final Pose2d m_measurement;
        private final Pose2d m_reference;

        public ManualSample(Pose2d measurement, Pose2d reference) {
            m_measurement = measurement;
            m_reference = reference;
        }

        @Override
        public Pose2d measurement() {
            return m_measurement;
        }

        @Override
        public Pose2d reference() {
            return m_reference;
        }

    }

    private final Config m_config = new Config();
    private final Supplier<Pose2d> m_measurements;
    private final Supplier<Twist2d> m_twister;

    public ManualDriveReferenceGenerator(Supplier<Pose2d> measurements, Supplier<Twist2d> twister) {
        m_measurements = measurements;
        m_twister = twister;
    }

    /** Ignores time. */
    @Override
    public ManualSample sample(double time) {
        Pose2d measurement = m_measurements.get();
        Twist2d twist = m_twister.get();
        return new ManualSample(measurement, measurement.exp(scale(twist)));
    }

    private Twist2d scale(Twist2d input) {
        return new Twist2d(m_config.scale * input.dx, m_config.scale * input.dy, m_config.scale * input.dtheta);
    }

}
