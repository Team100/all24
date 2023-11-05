package org.team100.frc2023.autonomous;


import org.team100.frc2023.subsystems.LaundryArm;

import edu.wpi.first.wpilibj2.command.Command;

/** Tells the arm to dump and waits for it to comply. */
public class Dump extends Command {
    private final LaundryArm m_arm;

    public Dump(LaundryArm arm) {
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.dump();
    }

    @Override
    public boolean isFinished() {
        return m_arm.atGoal();
    }
}
