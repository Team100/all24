package org.team100.frc2024.turret;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.util.Neo550Factory;

public class TurretCollection {
    private static final String kTurret = "Turret";

    private final OutboardAngularPositionServo m_turret;

    private TurretCollection(OutboardAngularPositionServo turret) {
        m_turret = turret;
    }

    public static TurretCollection get(
        LoggerFactory parent
    ) {
        LoggerFactory collectionLogger = parent.child(kTurret);
        switch (Identity.instance) {
            case FRC_100_ea4:
            //TODO get real identity, canID, maxVel, and maxAccel, also motorPhase may be wrong
            OutboardAngularPositionServo angularPositionServo = createAngularPositionServo(kTurret, collectionLogger,40,40, 1, 14, MotorPhase.REVERSE);
            angularPositionServo.setProfile(new TrapezoidProfile100(1, 1, 0.01));
            return new TurretCollection(angularPositionServo);
            case BLANK:
            default:
            OutboardAngularPositionServo simServo = getSimServo(collectionLogger); 
            simServo.setProfile(new TrapezoidProfile100(1, 1, 0.01));
            return new TurretCollection(simServo);
        }
    }

    private static OutboardAngularPositionServo getSimServo(LoggerFactory parent) {
        RotaryMechanism rotaryMechanism = Neo550Factory.simulatedRotaryMechanism(parent);
        return new OutboardAngularPositionServo(parent, rotaryMechanism, new CombinedEncoder(parent, new ProxyRotaryPositionSensor(rotaryMechanism), rotaryMechanism));
    }

    private static OutboardAngularPositionServo createAngularPositionServo(String name,
            LoggerFactory parent,
            int supplyLimit,
            int statorLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase) {
        LoggerFactory moduleLogger = parent.child(name);
        //TODO tune PID and feedforward
        Kraken6Motor kraken6Motor = new Kraken6Motor(moduleLogger, canID, motorPhase, supplyLimit, statorLimit, new PIDConstants(1), Feedforward100.makeKrakenTurret());
        RotaryMechanism rotaryMechanism = new RotaryMechanism(moduleLogger, kraken6Motor, new Talon6Encoder(moduleLogger, kraken6Motor), gearRatio);
        OutboardAngularPositionServo outboardAngularPositionServo = new OutboardAngularPositionServo(moduleLogger, rotaryMechanism, new CombinedEncoder(moduleLogger, new ProxyRotaryPositionSensor(rotaryMechanism), rotaryMechanism));
        return outboardAngularPositionServo;
    }

    public OutboardAngularPositionServo getTurret() {
        return m_turret;
    }
}
