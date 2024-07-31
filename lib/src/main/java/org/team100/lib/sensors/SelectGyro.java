package org.team100.lib.sensors;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class SelectGyro implements Gyro {
    private final Gyro m_whenTrue;
    private final Gyro m_whenFalse;
    private final BooleanSupplier m_selector;

    public SelectGyro(Gyro whenTrue, Gyro whenFalse, BooleanSupplier selector) {
        m_whenTrue = whenTrue;
        m_whenFalse = whenFalse;
        m_selector = selector;
    }

    @Override
    public Rotation2d getYawNWU() {
        if (m_selector.getAsBoolean()) {
            return m_whenTrue.getYawNWU();
        } else {
            return m_whenFalse.getYawNWU();
        }
    }

    @Override
    public double getYawRateNWU() {
        if (m_selector.getAsBoolean()) {
            return m_whenTrue.getYawRateNWU();
        } else {
            return m_whenFalse.getYawRateNWU();
        }
    }

    @Override
    public Rotation2d getPitchNWU() {
        if (m_selector.getAsBoolean()) {
            return m_whenTrue.getPitchNWU();
        } else {
            return m_whenFalse.getPitchNWU();
        }
    }

    @Override
    public Rotation2d getRollNWU() {
        if (m_selector.getAsBoolean()) {
            return m_whenTrue.getRollNWU();
        } else {
            return m_whenFalse.getRollNWU();
        }
    }

}
