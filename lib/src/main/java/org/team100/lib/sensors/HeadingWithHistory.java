package org.team100.lib.sensors;

import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * We usually ignore camera-derived heading and use gyro heading instead,
 * because the gyro is more accurate, but to do that, we need a time-aligned
 * heading. This class keeps history of heading measurements so we can
 * interpolate to get the right one.
 */
public class HeadingWithHistory implements HeadingInterface {
    private class InterpolationRecord implements Interpolatable<InterpolationRecord> {
        private final Rotation2d gyroAngle;

        private InterpolationRecord(Rotation2d gyro) {
            this.gyroAngle = gyro;
        }

        @Override
        public InterpolationRecord interpolate(InterpolationRecord endValue, double t) {
            if (t < 0) {
                return this;
            } else if (t >= 1) {
                return endValue;
            } else {
                Rotation2d gyroLerp = gyroAngle.interpolate(endValue.gyroAngle, t);
                return new InterpolationRecord(gyroLerp);
            }
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) {
                return true;
            }
            if (!(obj instanceof InterpolationRecord)) {
                return false;
            }
            InterpolationRecord ir = (InterpolationRecord) obj;
            return gyroAngle.equals(ir.gyroAngle);
        }

        @Override
        public int hashCode() {
            return Objects.hash(gyroAngle);
        }
    }

    private final HeadingInterface m_delegate;
    private static final double kBufferDuration = 1.5;
    private final TimeInterpolatableBuffer<InterpolationRecord> m_buffer = TimeInterpolatableBuffer
            .createBuffer(kBufferDuration);

    public HeadingWithHistory(HeadingInterface delegate) {
        m_delegate = delegate;
        // we'll call periodic
        CommandScheduler.getInstance().unregisterSubsystem(delegate);
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public Rotation2d getHeadingNWU() {
        return m_delegate.getHeadingNWU();
    }

    @Override
    public double getHeadingRateNWU() {
        return m_delegate.getHeadingRateNWU();
    }

    @Override
    public void periodic() {
        // get new measurements
        m_delegate.periodic();
        // store them
        m_buffer.addSample(
                Timer.getFPGATimestamp(),
                new InterpolationRecord(getHeadingNWU()));
    }

    /**
     * @param timestampSeconds fpga time
     */
    public Rotation2d sample(double timestampSeconds) {
        Optional<InterpolationRecord> sample = m_buffer.getSample(timestampSeconds);

        // this should really never happen
        if (sample.isEmpty())
            return null;

        return sample.get().gyroAngle;
    }

}
