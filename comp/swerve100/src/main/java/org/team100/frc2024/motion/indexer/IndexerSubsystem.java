package org.team100.frc2024.motion.indexer;

import org.team100.lib.controller.State100;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motor.drive.NeoDriveMotor;
import org.team100.lib.profile.Constraints100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    private final PositionServo<Distance> driveMotor;

    private final double kGearRatio = 1;
    private final double kWheelDiameter = 1;
    private final double kMaxVelM_S = 1;
    private final double kMaxAccelM_S2 = 1;

    //TODO GET THE RIGHT NUMBERS

    public IndexerSubsystem(String name, int canID) {
        driveMotor = ServoFactory.neoPositionServo(
                name,
                canID,
                false,
                kGearRatio,
                kWheelDiameter,
                kMaxVelM_S,
                kMaxAccelM_S2,
                new PIDController(1, 0, 0));

        // m_motor = new NeoDriveMotor(name, canID, true, 1, 0.1);
        // m_trapezoid = new TrapezoidProfile100(
        //         new Constraints100(4, 10), 0.05);
        // setpoint = new State100(0, 0, 0);
    }

    public void set(double value) {
        // setpoint = m_trapezoid.calculate(0.02, setpoint, new State100(value, 0, 0));
        driveMotor.setVelocity(value);
    }

    @Override
    public void periodic() {
        driveMotor.periodic();
    }
}
