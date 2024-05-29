package org.team100.field;

import java.util.Formatter;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Periodically displays the score, and also the state of the scorekeeper (e.g.
 * the amplification timer).
 */
public class ScoreDisplay {
    private static final String kReset = "\033[0m";
    private static final String kBoldBlue = "\033[1;37;44m";
    private static final String kBlue = "\033[0;37;44m";
    private static final String kBoldRed = "\033[1;37;41m";
    private static final String kRed = "\033[0;37;41m";
    private static final String kAmped = "   Amplified!   ";
    private static final String kBlank = "                ";

    private final ScheduledExecutorService m_scheduler;

    private final Scorekeeper m_scorekeeper;
    private final Score m_blue;
    private final Score m_red;
    private final SimulatedFMS m_fms;

    public ScoreDisplay(
            Scorekeeper scorekeeper,
            Score blueScore,
            Score redScore,
            SimulatedFMS fms) {
        m_scorekeeper = scorekeeper;
        m_blue = blueScore;
        m_red = redScore;
        m_fms = fms;
        m_scheduler = Executors.newSingleThreadScheduledExecutor();
    }

    public void start() {
        m_scheduler.scheduleAtFixedRate(this::periodic, 0, 1, TimeUnit.SECONDS);
    }

    public void periodic() {
        if (m_fms.isFinished()) {
            finalScore();
            m_scheduler.shutdown();
        } else {
            duringMatch();
        }
    }

    /** Like the audience display on the bottom of the screen. */
    public void duringMatch() {
        try {
            double matchTimeSec = m_fms.getMatchTime();
            int minutes = (int) matchTimeSec / 60;
            int seconds = (int) matchTimeSec % 60;
            StringBuilder b = new StringBuilder();
            Formatter f = new Formatter(b);
            if (m_scorekeeper.blueAmplified() > 0) {
                b.append(kReset);
                f.format(" %2.0f ", m_scorekeeper.blueAmplified());
                b.append(kBoldBlue);
                b.append(kAmped);
            } else { // maintain alignment
                b.append(kReset);
                b.append("    "); // score placeholder
                b.append(kBlank);
            }
            b.append(kBlue);
            b.append(" Blue ");
            b.append(kBoldBlue);
            f.format(" %3d ", m_blue.TotalScore());
            b.append(kReset);
            f.format(" %2d:%02d ", minutes, seconds);
            b.append(kBoldRed);
            f.format(" %3d ", m_red.TotalScore());
            b.append(kRed);
            b.append(" Red  ");
            if (m_scorekeeper.redAmplified() > 0) {
                b.append(kBoldRed);
                b.append(kAmped);
                b.append(kReset);
                f.format(" %2.0f ", m_scorekeeper.redAmplified());
            }
            b.append(kReset);
            System.out.println(b.toString());
            f.close();
        } catch (Throwable e) {
            e.printStackTrace();
        }
    }

    /**
     * Like The Blue Alliance, but with blue on the left.
     * TODO: add endgame section
     */
    public void finalScore() {
        final int ampPoint = 1;
        final int notAmpedSpeakerPoint = 2;
        final int ampedSpeakerPoint = 5;

        final int blueAutoLeave = 0;
        final int redAutoLeave = 0;
        final int blueAutoAmps = m_blue.AutoAmpNoteCount;
        final int redAutoAmps = m_red.AutoAmpNoteCount;
        final int blueAutoSpeakers = m_blue.AutoSpeakerNoteCount;
        final int redAutoSpeakers = m_red.AutoSpeakerNoteCount;

        final int blueAmps = m_blue.TeleopAmpNoteCount;
        final int redAmps = m_red.TeleopAmpNoteCount;
        final int blueNotAmpedSpeakers = m_blue.TeleopSpeakerNoteCountNotAmplified;
        final int blueAmpedSpeakers = m_blue.TeleopSpeakerNoteCountAmplified;
        final int redNotAmpedSpeakers = m_red.TeleopSpeakerNoteCountNotAmplified;
        final int redAmpedSpeakers = m_red.TeleopSpeakerNoteCountAmplified;

        final StringBuilder b = new StringBuilder();
        final Formatter f = new Formatter(b);

        b.append(kBlue);
        f.format("    %2d   ", blueAutoLeave);
        b.append(kReset);
        b.append("         Auto Leave        ");
        // .......012345678901234509876543210        
        b.append(kRed);
        f.format("    %2d   ", redAutoLeave);
        b.append(kReset);
        b.append("\n");

        b.append(kBlue);
        f.format("    %2d   ", blueAutoAmps);
        b.append(kReset);
        b.append("    Auto Amp Note Count    ");
        // .......012345678901234509876543210
        b.append(kRed);
        f.format("    %2d   ", redAutoAmps);
        b.append(kReset);
        b.append("\n");

        b.append(kBlue);
        f.format("    %2d   ", blueAutoSpeakers);
        b.append(kReset);
        b.append("  Auto Speaker Note Count  ");
        // .......012345678901234509876543210
        b.append(kRed);
        f.format("    %2d   ", redAutoSpeakers);
        b.append(kReset);
        b.append("\n");

        final int blueAutoNotePoints = blueAutoAmps * 2 + blueAutoSpeakers * 5;

        final int redAutoNotePoints = redAutoAmps * 2 + redAutoSpeakers * 5;

        b.append(kBlue);
        f.format("    %2d   ", blueAutoNotePoints);
        b.append(kReset);
        b.append("      Auto Note Points     ");
        // .......012345678901234509876543210
        b.append(kRed);
        f.format("    %2d   ", redAutoNotePoints);
        b.append(kReset);
        b.append("\n");

        final int blueTotalAuto = blueAutoLeave + blueAutoNotePoints;
        final int redTotalAuto = redAutoLeave + redAutoNotePoints;

        b.append(kBlue);
        f.format("    %2d   ", blueTotalAuto);
        b.append(kReset);
        b.append("         Total Auto        ");
        // .......012345678901234509876543210
        b.append(kRed);
        f.format("    %2d   ", redTotalAuto);
        b.append(kReset);
        b.append("\n");

        b.append(kBlue);
        f.format("    %2d   ", blueAmps);
        b.append(kReset);
        b.append("   Teleop Amp Note Count   ");
        // .......012345678901234509876543210
        b.append(kRed);
        f.format("    %2d   ", redAmps);
        b.append(kReset);
        b.append("\n");

        b.append(kBlue);
        f.format(" %2d / %2d ", blueNotAmpedSpeakers, blueAmpedSpeakers);
        b.append(kReset);
        b.append(" Teleop Speaker Note Count ");
        // .......012345678901234509876543210
        b.append(kRed);
        f.format(" %2d / %2d ", redNotAmpedSpeakers, redAmpedSpeakers);
        b.append(kReset);
        b.append("\n");

        final int blueTeleopTotal = blueAmps * ampPoint
                + blueNotAmpedSpeakers * notAmpedSpeakerPoint
                + blueAmpedSpeakers * ampedSpeakerPoint;
        final int redTeleopTotal = redAmps * ampPoint
                + redNotAmpedSpeakers * notAmpedSpeakerPoint
                + redAmpedSpeakers * ampedSpeakerPoint;
        b.append(kBoldBlue);
        f.format("    %2d   ", blueTeleopTotal);
        b.append(kReset);
        b.append("     Teleop Note Points    ");
        // .......012345678901234509876543210
        b.append(kBoldRed);
        f.format("    %2d   ", redTeleopTotal);
        b.append(kReset);
        b.append("\n");

        final int blueTotalTeleop = blueTeleopTotal;
        final int redTotalTeleop = redTeleopTotal;

        b.append(kBoldBlue);
        f.format("    %2d   ", blueTotalTeleop);
        b.append(kReset);
        b.append("        Total Teleop       ");
        // .......012345678901234509876543210
        b.append(kBoldRed);
        f.format("    %2d   ", redTotalTeleop);
        b.append(kReset);
        b.append("\n");

        final int blueTotalScore = blueTotalAuto + blueTotalTeleop;
        final int redTotalScore = redTotalAuto + redTotalTeleop;

        b.append(kBoldBlue);
        f.format("    %2d   ", blueTotalScore);
        b.append(kReset);
        b.append("        Total Score        ");
        // .......012345678901234509876543210
        b.append(kBoldRed);
        f.format("    %2d   ", redTotalScore);
        b.append(kReset);
        b.append("\n");

        System.out.println(b.toString());
        f.close();

    }

}
