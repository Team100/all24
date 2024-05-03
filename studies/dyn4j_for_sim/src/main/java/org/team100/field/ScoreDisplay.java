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

    // to alternate amping, to see what it looks like
    // TODO: remove
    private boolean amped = false;

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
            // to work out the format
            // double blueAmpTime = 7.5;
            // double redAmpTime = 10.0;
            // int fakeBlueScore = 63;
            // int fakeRedScore = 82;
            double matchTimeSec = m_fms.getMatchTime();
            int minutes = (int) matchTimeSec / 60;
            int seconds = (int) matchTimeSec % 60;
            StringBuilder b = new StringBuilder();
            Formatter f = new Formatter(b);
            if (amped) {
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
            if (amped) {
                b.append(kBoldRed);
                b.append(kAmped);
                b.append(kReset);
                f.format(" %2.0f ", m_scorekeeper.redAmplified());
            }
            b.append(kReset);
            System.out.println(b.toString());
            f.close();
            amped ^= true;
        } catch (Throwable e) {
            e.printStackTrace();
        }
    }

    /** Like The Blue Alliance, but with blue on the left. */
    public void finalScore() {
        int ampPoint = 1;
        int notAmpedSpeakerPoint = 2;
        int ampedSpeakerPoint = 5;

        int redAmps = 7;
        int blueAmps = 7;
        int redNotAmpedSpeakers = 2;
        int redAmpedSpeakers = 10;
        int blueNotAmpedSpeakers = 10;
        int blueAmpedSpeakers = 4;

        StringBuilder b = new StringBuilder();
        Formatter f = new Formatter(b);

        b.append(kBlue);
        f.format("    %2d   ", blueAmps);
        b.append(kReset);
        b.append("   Teleop Amp Note Count   ");
        b.append(kRed);
        f.format("    %2d   ", redAmps);
        b.append(kReset);
        b.append("\n");

        b.append(kBlue);
        f.format(" %2d / %2d ", blueNotAmpedSpeakers, blueAmpedSpeakers);
        b.append(kReset);
        b.append(" Teleop Speaker Note Count ");
        b.append(kRed);
        f.format(" %2d / %2d ", redNotAmpedSpeakers, redAmpedSpeakers);
        b.append(kReset);
        b.append("\n");

        int blueTotal = blueAmps * ampPoint
                + blueNotAmpedSpeakers * notAmpedSpeakerPoint
                + blueAmpedSpeakers * ampedSpeakerPoint;
        int redTotal = redAmps * ampPoint
                + redNotAmpedSpeakers * ampedSpeakerPoint
                + redAmpedSpeakers * ampedSpeakerPoint;
        b.append(kBoldBlue);
        f.format("    %2d   ", blueTotal);
        b.append(kReset);
        b.append("     Teleop Note Points    ");
        b.append(kBoldRed);
        f.format("    %2d   ", redTotal);
        b.append(kReset);

        System.out.println(b.toString());
        f.close();

    }

}
