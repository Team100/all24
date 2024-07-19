package org.team100.frc2024.motion.amp;

import java.util.OptionalDouble;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.State100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.SupplierLogger;

/**
 * Set the amp pivot state, including current limit, position, and velocity.
 * 
 * These can be used in sequence, to allow varying levels of speed and effort in
 * a single motion: make the goal velocity of the first one equal to the cruise
 * velocity of the second, for example.
 */
public class AmpState extends Command100 {
    private static final double kToleranceRad = 0.05;
    private final AmpPivot m_pivot;
    private final State100 m_goal;
    private final Profile100 m_profile;
    private final double m_torqueLimit;

    public AmpState(
            SupplierLogger parent,
            AmpPivot pivot,
            State100 goal,
            double maxVelRad_S,
            double maxAccelRad_S2,
            double torqueLimit) {
        super(parent);
        m_pivot = pivot;
        m_goal = goal;
        m_profile = new TrapezoidProfile100(maxVelRad_S, maxAccelRad_S2, kToleranceRad);
        m_torqueLimit = torqueLimit;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize100() {
        m_pivot.setTorqueLimit(m_torqueLimit);
        m_pivot.setProfile(m_profile);
        m_pivot.reset();
    }

    @Override
    public void execute100(double dt) {

        m_pivot.setAmpState(m_goal);
    }

    @Override
    public boolean isFinished() {
        OptionalDouble opt = m_pivot.getPositionRad();
        if (opt.isEmpty())
            return true;
        double positionRad = opt.getAsDouble();
        return Math.abs(m_goal.x() - positionRad) < kToleranceRad;
    }

}
