package org.team100.frc2024.motion.intake;

import org.team100.lib.controller.State100;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motor.drive.NeoDriveMotor;
import org.team100.lib.profile.Constraints100;
import org.team100.lib.profile.TrapezoidProfile100;

import com.ctre.phoenix6.mechanisms.DifferentialMechanism.DisabledReason;

import edu.wpi.first.math.controller.PIDController;
import org.team100.lib.units.Distance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    //TODO GET THE RIGHT NUMBERS
    private static final double kGearRatio = 1;
    private static final double kWheelDiameter = 1;
    private static final double kMaxVelM_S = 5;
    private static final double kMaxAccelM_S2 = 5;

    private final PositionServo<Distance> topRoller;
    private final PositionServo<Distance> bottomRoller;

    public IntakeSubsystem(String name1, String name2, int canID1, int canID2) {
        topRoller = ServoFactory.neoPositionServo(
                name1,
                canID1,
                false,
                kGearRatio,
                kWheelDiameter,
                kMaxVelM_S,
                kMaxAccelM_S2,
                new PIDController(1, 0, 0));

        bottomRoller = ServoFactory.neoPositionServo(
                name2,
                canID2,
                false,
                kGearRatio,
                kWheelDiameter,
                kMaxVelM_S,
                kMaxAccelM_S2,
                new PIDController(1, 0, 0));
    }

    public void set(double value) {
        // setpoint = m_trapezoid.calculate(0.02, setpoint, new State100(value, 0, 0));
        topRoller.setVelocity(value);
        bottomRoller.setVelocity(value);
    }

    @Override
    public void periodic() {
        topRoller.periodic();
        bottomRoller.periodic();
    }
}
