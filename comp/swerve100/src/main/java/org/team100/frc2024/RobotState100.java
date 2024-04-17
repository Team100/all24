package org.team100.frc2024;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

public class RobotState100 {
    private static final Telemetry t = Telemetry.get();

    public enum ShooterState100 {
        DEFAULTSHOOT, STOP, TEST, LOB
    }

    public enum AmpState100 {
        UP, OUTTAKE, NONE
    }

    public enum IntakeState100 {
        INTAKE, STOP
    }

    public enum FeederState100 {
        FEED, STOP
    }

    private static ShooterState100 currentShooterState = ShooterState100.STOP;
    private static AmpState100 currentAmpState = AmpState100.NONE;
    private static IntakeState100 currentIntakeState = IntakeState100.STOP;
    private static FeederState100 currentFeederState = FeederState100.STOP;

    public static void changeShooterState(ShooterState100 state) {
        currentShooterState = state;
    }

    public static ShooterState100 getShooterState() {
        return currentShooterState;
    }

    public static void changeAmpState(AmpState100 state) {
        currentAmpState = state;
    }

    public static AmpState100 getAmpState() {
        return currentAmpState;
    }

    public static void changeIntakeState(IntakeState100 state) {
        t.log(Level.DEBUG, "ChangeIntakeState", "state", state);
        currentIntakeState = state;
    }

    public static IntakeState100 getIntakeState() {
        return currentIntakeState;
    }

    public static void changeFeederState(FeederState100 state) {
        currentFeederState = state;
    }

    public static FeederState100 getFeederState() {
        return currentFeederState;
    }

}
