package org.team100.frc2024.drivetrain;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motion.SimpleLinearMechanism;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.components.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550CANSparkMotor;
import org.team100.lib.telemetry.SupplierLogger;

public class TankModule100 {
    private static final double kWheelDiameterM = 0.0975; // 0.1015

    public static LinearVelocityServo get(
            String name,
            SupplierLogger parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase) {
        SupplierLogger moduleLogger = parent.child(name);
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
            moduleLogger, 
            canID,
            motorPhase,
            currentLimit,
            Feedforward100.makeNeo550(),
            new PIDConstants(1));
        LinearMechanism mech = new SimpleLinearMechanism(
                motor,
                new CANSparkEncoder(moduleLogger, motor),
                gearRatio,
                kWheelDiameterM);
        return new OutboardLinearVelocityServo(moduleLogger, mech);
    }
}
