package org.team100.frc2024.motion.amp;

import org.team100.frc2024.RobotState100;

import edu.wpi.first.wpilibj2.command.Command;

public class AmpDefault extends Command {
    AmpSubsystem m_amp;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_amp.reset();
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // switch(RobotState100.getRobotState()){
    //     case AMPING:
    //         switch(RobotState100.getAmpState()){
    //             case UP:
    //                 System.out.println("AMP IS GOIIING UPPPPPPPPPP");
    //                 m_amp.setAmpPosition(0.1);
    //                 m_amp.stopFeed();
    //                 break;
    //             case DOWN:
    //                 m_amp.setAmpPosition(-0.1);
    //                 m_amp.stopFeed();
    //                 break;
    //             case NONE:
    //                 m_amp.setAmpPosition(0);
    //                 m_amp.stopFeed();

    //             default:
    //         }
    //         // System.out.println("AHHHHHHHHHHHHHHHHHHHHHHH");

    //     default:
    //         m_amp.setAmpPosition(0);

    //         m_amp.stopFeed();

    // }


        //0.34
    switch(RobotState100.getAmpState()){
                case UP:
                    m_amp.setAmpPosition(0.7);
                    // m_amp.setDutyCycle(1);

                    m_amp.driveFeeder(0);
                    break;
                case DOWN:
                    m_amp.setDutyCycle(-0.3);
                    m_amp.driveFeeder(0);
                    break;
                case OUTTAKE:
                    m_amp.driveFeeder(1);
                    break;
                case NONE:
                    m_amp.setDutyCycle(0);
                    m_amp.driveFeeder(0);
                default:
    }


  

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
