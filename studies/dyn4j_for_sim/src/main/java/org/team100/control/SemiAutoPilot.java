package org.team100.control;

import java.util.function.Supplier;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Combines velocity input (from a human) with acceleration input (repel from
 * obstacles, attract to goals).
 */
public class SemiAutoPilot implements Pilot {
    private final Pilot m_pilot;
    private final Supplier<FieldRelativeAcceleration> m_accel;

    private long timeMicros;

    public SemiAutoPilot(Pilot pilot, Supplier<FieldRelativeAcceleration> accel) {
        m_pilot = pilot;
        m_accel = accel;
        timeMicros = RobotController.getFPGATime();
    }

    @Override
    public FieldRelativeVelocity driveVelocity() {
        FieldRelativeVelocity driverVelocity = m_pilot.driveVelocity();
        FieldRelativeAcceleration acceleration = m_accel.get();
        long nowMicros = RobotController.getFPGATime();
        double dtSec = (double) (nowMicros - timeMicros) / 1000000;
        timeMicros = nowMicros;
        FieldRelativeVelocity dv = acceleration.integrate(dtSec);
        return driverVelocity.plus(dv);
    }

    @Override
    public boolean intake() {
        return m_pilot.intake();
    }

    @Override
    public boolean outtake() {
        return m_pilot.outtake();
    }

    @Override
    public boolean shoot() {
        return m_pilot.shoot();
    }

    @Override
    public boolean lob() {
        return m_pilot.lob();
    }

    @Override
    public boolean amp() {
        return m_pilot.amp();
    }

    @Override
    public boolean rotateToShoot() {
        return m_pilot.rotateToShoot();
    }

    @Override
    public boolean driveToSpeaker() {
        return m_pilot.driveToSpeaker();
    }

}
