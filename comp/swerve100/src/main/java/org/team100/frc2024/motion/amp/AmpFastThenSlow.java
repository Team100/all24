package org.team100.frc2024.motion.amp;

import org.team100.lib.controller.State100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.SupplierLogger;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AmpFastThenSlow extends SequentialCommandGroup {
    private static final double kTolerance = 0.05; // 3 degrees
    private static final double kFastVelocityRad_S = 5;
    private static final double kSlowVelocityRad_S = 1;
    private static final double kFastAccelRad_S2 = 6;
    private static final double kSlowAccelRad_S2 = 6;
    private static final double kFastTorqueLimitNm = 50;
    private static final double kSlowTorqueLimitNm = 10;
    private static final TrapezoidProfile100 kFastProfile = new TrapezoidProfile100(kFastVelocityRad_S,
            kFastAccelRad_S2, kTolerance);
    private static final TrapezoidProfile100 kSlowProfile = new TrapezoidProfile100(kSlowVelocityRad_S,
            kSlowAccelRad_S2, kTolerance);

    public AmpFastThenSlow(
            SupplierLogger parent,
            AmpPivot pivot,
            double switchRad,
            double goalRad) {
        addCommands(
                // first go fast to the switch point at the slow velocity
                new AmpState(
                        parent,
                        pivot,
                        new State100(
                                switchRad,
                                switchRad < goalRad ? kSlowVelocityRad_S : -kSlowVelocityRad_S),
                        kFastProfile,
                        kFastTorqueLimitNm,
                        kTolerance,
                        false), // don't hold
                // then go slow to the goal at rest
                new AmpState(
                        parent,
                        pivot,
                        new State100(goalRad, 0.0),
                        kSlowProfile,
                        kSlowTorqueLimitNm,
                        kTolerance,
                        true)); // hold at the end
    }
}
