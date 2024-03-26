package org.team100.frc2024.motion;


import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.motion.amp.AmpSubsystem;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FeedCommand extends Command {
    private static final double shooterAngle = 0.110273;
  
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final AmpSubsystem m_amp;
    private final FeederSubsystem m_feeder;
    private final SensorInterface m_sensors;

    boolean noteInAmp = false;
    boolean shooterIsInPosition = false;
    boolean finished = false;

    Timer m_timer = new Timer();

    public FeedCommand(Intake intake, Shooter shooter, AmpSubsystem amp, FeederSubsystem feeder, SensorInterface sensors) {
        m_intake = intake;
        m_shooter = shooter;
        m_amp = amp;
        m_feeder = feeder;
        m_sensors = sensors;
        addRequirements(m_amp, m_intake, m_shooter, m_feeder);
    }
  
    @Override
    public void initialize() {
        shooterIsInPosition = false;
        noteInAmp = false;
        m_timer.reset();
        finished = false;
    }

    @Override
    public void execute() {
        m_shooter.setPivotPosition(shooterAngle);

        if (Math.abs(m_shooter.getPivotPosition() - shooterAngle) > 0.1) {
            m_shooter.setPivotPosition(shooterAngle);
        } else {
            shooterIsInPosition = true;
        }
        
        if(shooterIsInPosition){
            if(m_sensors.getAmpSensor() == false){
                noteInAmp = true;
            } else {
                m_feeder.feed();
                m_intake.intake();
                m_shooter.feed();
                m_amp.driveFeeder(-1);
            }
        }

        if(noteInAmp){
            m_timer.start();
        }

        if(m_timer.get() >= 0.1){
            finished = true;
        }
    }
  
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_amp.driveFeeder(0);
        shooterIsInPosition = false;
        finished = false;
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
