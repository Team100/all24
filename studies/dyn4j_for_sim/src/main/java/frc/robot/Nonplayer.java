package frc.robot;

public abstract class Nonplayer extends RobotBody {
    enum Goal {
        PICK, SCORE
    }

    Goal m_goal;
}
