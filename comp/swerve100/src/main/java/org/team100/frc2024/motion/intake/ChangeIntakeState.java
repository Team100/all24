package org.team100.frc2024.motion.intake;

import org.team100.frc2024.SensorInterface;

import edu.wpi.first.wpilibj2.command.Command;

public class ChangeIntakeState extends Command {
    private final Intake m_intake;
    private final SensorInterface m_sensors;

    public ChangeIntakeState(Intake intake, SensorInterface sensors) {
        m_intake = intake;
        m_sensors = sensors;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.intakeSmart();
    }

    @Override
    public void execute() {
        if(m_sensors.getFeederSensor()){
            m_intake.intake();
        }
    }


    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
