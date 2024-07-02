package org.team100.control;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

/** Proxy auton during autonomous period, otherwise teleop. */
public class SelectorPilot implements Pilot {
    BooleanSupplier m_selector;
    private final Pilot m_true;
    private final Pilot m_false;

    public SelectorPilot(BooleanSupplier selector, Pilot ptrue, Pilot pfalse) {
        m_selector = selector;
        m_true = ptrue;
        m_false = pfalse;
    }

    public static SelectorPilot autonSelector(Pilot ptrue, Pilot pfalse) {
        return new SelectorPilot(DriverStation::isAutonomous, ptrue, pfalse);
    }

    @Override
    public FieldRelativeVelocity driveVelocity() {
        return select((Pilot::driveVelocity));
    }

    @Override
    public boolean intake() {
        return select(Pilot::intake);
    }

    @Override
    public boolean outtake() {
        return select(Pilot::outtake);
    }

    @Override
    public boolean shoot() {
        return select(Pilot::shoot);
    }

    @Override
    public boolean lob() {
        return select(Pilot::lob);
    }

    @Override
    public boolean amp() {
        return select(Pilot::amp);
    }

    @Override
    public boolean rotateToShoot() {
        return select(Pilot::rotateToShoot);
    }

    @Override
    public boolean scoreSpeaker() {
        return select(Pilot::scoreSpeaker);
    }

    @Override
    public boolean scoreAmp() {
        return select(Pilot::scoreAmp);
    }

    @Override
    public boolean driveToSource() {
        return select(Pilot::driveToSource);
    }

    @Override
    public boolean pass() {
        return select(Pilot::pass);
    }

    @Override
    public boolean driveToNote() {
        return select(Pilot::driveToNote);
    }

    @Override
    public boolean shootCommand() {
        return select(Pilot::shootCommand);
    }

    @Override
    public boolean defend() {
        return select(Pilot::defend);
    }

    @Override
    public boolean driveToStaged() {
        return select(Pilot::driveToStaged);
    }

    @Override
    public Pose2d shootingLocation() {
        return select(Pilot::shootingLocation);
    }

    @Override
    public int goalNote() {
        return select(Pilot::goalNote);
    }

    @Override
    public void begin() {
        select(Pilot::begin);
    }

    @Override
    public void reset() {
        select(Pilot::reset);
    }

    @Override
    public void periodic() {
        select(Pilot::periodic);
    }

    private <T> T select(Function<Pilot, T> fn) {
        if (m_selector.getAsBoolean()) {
            return fn.apply(m_true);
        } else {
            return fn.apply(m_false);
        }
    }

    private void select(Consumer<Pilot> fn) {
        if (m_selector.getAsBoolean()) {
            fn.accept(m_true);
        } else {
            fn.accept(m_false);
        }
    }

}
