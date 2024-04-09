package org.team100.frc2024.motion;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase implements Glassy {
    private final String m_name;
    private final Telemetry t = Telemetry.get();
    private final PWM feedRoller;
    private final double kFeederVelocityM_S = 30;

    public FeederSubsystem(int feederID) {
        int feederLimit = 40;

        m_name = Names.name(this);
        SysParam feederParams = SysParam.limitedNeoVelocityServoSystem(1, 0.1, 30, 40, -40);


        switch (Identity.instance) {
            case COMP_BOT:
                // TODO tune kV
                feedRoller = new PWM(3);
                break;
            case BLANK:
            default:
                feedRoller = new PWM(3);
        }
    }

    public void starve() {
        feedRoller.setSpeed(-0.2);
    }

    public void feed() {
        feedRoller.setSpeed(0.8);

    }

    public void intake() {
        feedRoller.setSpeed(0.5);

    }

    public void outtake() {
        feedRoller.setSpeed(-0.1);

    }

    public void stop() {
        // System.out.println("STOPING FEED" + Timer.getFPGATimestamp());

        feedRoller.setSpeed(0);

        // feedRoller.setSpeed(0.5);
    }



    @Override
    public void periodic() {
        // feedRoller.periodic();
        // feedRoller.setSpeed(0.5);

        t.log(org.team100.lib.telemetry.Telemetry.Level.DEBUG, "FEEDER", "GET SPEED" , feedRoller.getSpeed());
    }

    @Override
    public String getGlassName() {
        return "Feeder";
    }
}
