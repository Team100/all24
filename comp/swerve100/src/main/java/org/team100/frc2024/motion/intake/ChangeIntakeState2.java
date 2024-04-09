package org.team100.frc2024.motion.intake;

import org.team100.frc2024.SensorInterface;

import edu.wpi.first.wpilibj2.command.Command;

public class ChangeIntakeState2 extends Command {
    private final Intake m_intake;
    private final SensorInterface m_sensors;
    private boolean finished = false;

    public ChangeIntakeState2(Intake intake, SensorInterface sensors) {
        m_intake = intake;
        m_sensors = sensors;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        finished = false;
        m_intake.intakeSmart();
    }

    @Override
    public void execute() {
        if(m_sensors.getFeederSensor()){
            m_intake.intake();
        } else {
            finished = true;
        }
    }


    @Override
    public void end(boolean interrupted) {
        finished = false;

        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
