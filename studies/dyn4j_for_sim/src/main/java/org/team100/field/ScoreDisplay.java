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

    private static void print(StringBuilder b, String name, int blue, int red) {
        print(b, name, String.format("    %2d   ", blue), String.format("    %2d   ", red));
    }

    private static void print(StringBuilder b, String name, int blue1, int blue2, int red1, int red2) {
        print(b, name, String.format(" %2d / %2d ", blue1, blue2), String.format(" %2d / %2d ", red1, red2));
    }

    private static void print(StringBuilder b, String name, String blue, String red) {
        int nameLength = name.length();
        final int totalLength = 27;
        int initialPad = (totalLength - nameLength) / 2;
        int finalPad = totalLength - nameLength - initialPad;
        b.append(kBlue);
        b.append(blue);
        b.append(kReset);
        for (int i = 0; i < initialPad; ++i) {
            b.append(" ");
        }
        b.append(name);
        for (int i = 0; i < finalPad; ++i) {
            b.append(" ");
        }
        b.append(kRed);
        b.append(red);
        b.append(kReset);
        b.append("\n");
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

        final int blueAutoNotePoints = blueAutoAmps * 2 + blueAutoSpeakers * 5;
        final int redAutoNotePoints = redAutoAmps * 2 + redAutoSpeakers * 5;
        final int blueTotalAuto = blueAutoLeave + blueAutoNotePoints;
        final int redTotalAuto = redAutoLeave + redAutoNotePoints;

        final int blueTeleopTotal = blueAmps * ampPoint
                + blueNotAmpedSpeakers * notAmpedSpeakerPoint
                + blueAmpedSpeakers * ampedSpeakerPoint;
        final int redTeleopTotal = redAmps * ampPoint
                + redNotAmpedSpeakers * notAmpedSpeakerPoint
                + redAmpedSpeakers * ampedSpeakerPoint;

        final int blueTotalTeleop = blueTeleopTotal;
        final int redTotalTeleop = redTeleopTotal;

        final int blueTotalScore = blueTotalAuto + blueTotalTeleop;
        final int redTotalScore = redTotalAuto + redTotalTeleop;

        final StringBuilder b = new StringBuilder();

        print(b, "Auto Leave", blueAutoLeave, redAutoLeave);
        print(b, "Auto Amp Note Count", blueAutoAmps, redAutoAmps);
        print(b, "Auto Speaker Note Count", blueAutoSpeakers, redAutoSpeakers);
        print(b, "Auto Note Points", blueAutoNotePoints, redAutoNotePoints);
        print(b, "Total Auto", blueTotalAuto, redTotalAuto);
        print(b, "Teleop Amp Note Count", blueAmps, redAmps);
        print(b, "Teleop Speaker Note Count", blueNotAmpedSpeakers, blueAmpedSpeakers, redNotAmpedSpeakers,
                redAmpedSpeakers);
        print(b, "Teleop Note Points", blueTeleopTotal, redTeleopTotal);
        print(b, "Robot 1 Endgame", 0, 0);
        print(b, "Robot 2 Endgame", 0, 0);
        print(b, "Robot 3 Endgame", 0, 0);
        print(b, "Harmony Points", 0, 0);
        print(b, "Trap Points", 0, 0);
        print(b, "Total Teleop", blueTotalTeleop, redTotalTeleop);
        print(b, "Coopertition Criteria Met", 0, 0);
        print(b, "Melody Bonus", 0, 0);
        print(b, "Ensemble Bonus", 0, 0);
        print(b, "Fouls / Tech Fouls", 0, 0, 0, 0);
        print(b, "Adjustments", 0, 0);
        print(b, "Total Score", blueTotalScore, redTotalScore);
        print(b, "Ranking Points", 0, 0);

        System.out.println(b.toString());
    }

}
