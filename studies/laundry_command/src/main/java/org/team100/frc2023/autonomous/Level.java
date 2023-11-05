package org.team100.frc2023.autonomous;


import org.team100.frc2023.subsystems.LaundryArm;

import edu.wpi.first.wpilibj2.command.Command;

/** Tells the arm to level and waits for it to comply. */
public class Level extends Command {
    private final LaundryArm m_arm;

    public Level(LaundryArm arm) {
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.level();
    }

    @Override
    public boolean isFinished() {
        return m_arm.atGoal();
    }
}