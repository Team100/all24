package org.team100.frc2024.motion.shooter;

import org.team100.lib.controller.State100;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.ServoFactory;

import org.team100.lib.units.Distance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrumShooter extends ShooterFactory {
    private static final double kGearRatio = 20.0;
    private static final double kWinchDiameterM = 0.01;
    private static final double kMaxVelM_S = 8;
    private static final double kMaxAccelM_S2 = 30;

    private final PositionServo<Distance> topRoller;
    private final PositionServo<Distance> bottomRoller;

    public DrumShooter(String name1, String name2, int canID1, int canID2) {

        topRoller = ServoFactory.neoPositionServo(
                name1,
                canID1,
                false,
                kGearRatio,
                kWinchDiameterM,
                kMaxVelM_S,
                kMaxAccelM_S2,
                new PIDController(1, 0, 0));

        bottomRoller = ServoFactory.neoPositionServo(
                name2,
                canID2,
                false,
                kGearRatio,
                kWinchDiameterM,
                kMaxVelM_S,
                kMaxAccelM_S2,
                new PIDController(1, 0, 0));
        
    }

    public void set(double value) {
        topRoller.setVelocity(value);
        bottomRoller.setVelocity(value);
    }

    @Override
    public void periodic() {
        topRoller.periodic();
        bottomRoller.periodic();
    }
}
