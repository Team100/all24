package org.team100.frc2024.motion.amp;

import java.util.Map;
import java.util.function.Supplier;

import org.team100.lib.controller.State100;
import org.team100.lib.telemetry.SupplierLogger;

import edu.wpi.first.wpilibj2.command.SelectCommand;

public class AmpFastThenSlow extends SelectCommand<Boolean> {
    private static final double kFastVelocityRad_S = 2;
    private static final double kSlowVelocityRad_S = 0.2;
    private static final double kAccelRad_S2 = 2;
    private static final double kFastTorqueLimitNm = 2;
    private static final double kSlowTorqueLimitNm = 1;

    public AmpFastThenSlow(
            SupplierLogger parent,
            AmpPivot pivot,
            double switchRad,
            double goalRad) {
        super(Map.of(
                // first go fast to the switch point at the slow velocity
                true, new AmpState(
                        parent,
                        pivot,
                        new State100(switchRad, kSlowVelocityRad_S),
                        kFastVelocityRad_S,
                        kAccelRad_S2,
                        kFastTorqueLimitNm),
                // then go slow to the goal at rest
                false, new AmpState(
                        parent,
                        pivot,
                        new State100(goalRad, 0.0),
                        kSlowVelocityRad_S,
                        kAccelRad_S2,
                        kSlowTorqueLimitNm)),
                selector(pivot, goalRad, switchRad));
    }

    /**
     * Supply true if position is far from the goal.
     * If the measurement is broken, supply false.
     */
    private static Supplier<Boolean> selector(AmpPivot pivot, double goal, double switchPt) {
        return () -> {
            double pos = pivot.getPositionRad().orElse(goal);
            if (goal < switchPt) {
                // heading down, go fast if we're still above the switching point.
                return pos > switchPt;
            } else {
                // heading up, go fast if we're still below the switching point.
                return pos < switchPt;
            }
        };

    }

}
