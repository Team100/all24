package org.team100.field;

/**
 * A record of scoring events for one alliance during one match.
 * 
 * Does not implement coopertition.
 */
public class Score {
    enum Endgame {
        Nothing(0),
        Park(1),
        Onstage(3),
        Spotlit(4); // onstage with spotlight

        int Points;

        private Endgame(int p) {
            Points = p;
        }

        public boolean onstage() {
            return Points > 1;
        }
    }

    Score opponent;

    boolean AutoLeave;
    int AutoAmpNoteCount;
    int AutoSpeakerNoteCount;
    int TeleopAmpNoteCount;
    int TeleopSpeakerNoteCountNotAmplified;
    int TeleopSpeakerNoteCountAmplified;
    Endgame Robot1Endgame = Endgame.Nothing;
    Endgame Robot2Endgame = Endgame.Nothing;
    Endgame Robot3Endgame = Endgame.Nothing;
    boolean Harmony;
    int trap;
    int Fouls;
    int TechFouls;
    int Adjustments;

    int AutoNotePoints() {
        return 2 * AutoAmpNoteCount
                + 5 * AutoSpeakerNoteCount;
    }

    int TotalAuto() {
        return AutoLeave ? 2 : 0 + AutoNotePoints();
    }

    int TeleopNotePoints() {
        return TeleopAmpNoteCount
                + 2 * TeleopSpeakerNoteCountNotAmplified
                + 5 * TeleopSpeakerNoteCountAmplified;
    }

    int Robot1StagePoints() {
        return Robot1Endgame.Points;
    }

    int Robot2StagePoints() {
        return Robot2Endgame.Points;
    }

    int Robot3StagePoints() {
        return Robot3Endgame.Points;
    }

    int HarmonyPoints() {
        return Harmony ? 2 : 0;
    }

    int TrapPoints() {
        return 3 * trap;
    }

    int StagePoints() {
        return Robot1StagePoints()
                + Robot2StagePoints()
                + Robot3StagePoints()
                + HarmonyPoints()
                + TrapPoints();
    }

    int TotalTeleop() {
        return TeleopNotePoints() + StagePoints();
    }

    boolean CoopertitionCriteriaMet() {
        // TODO: implement "coopertition button" somehow.
        // true if coopertition button used by both alliances in the first 45 sec of
        // teleop.
        return false;
    }

    int CoopertitionBonus() {
        return CoopertitionCriteriaMet() ? 1 : 0;
    }

    boolean MelodyBonus() {
        return TeleopAmpNoteCount
                + TeleopSpeakerNoteCountNotAmplified
                + TeleopSpeakerNoteCountAmplified >= (CoopertitionCriteriaMet() ? 15 : 18);
    }

    int OnstageCount() {
        return (Robot1Endgame.onstage() ? 1 : 0)
                + (Robot2Endgame.onstage() ? 1 : 0)
                + (Robot3Endgame.onstage() ? 1 : 0);
    }

    boolean EnsembleBonus() {
        return StagePoints() >= 10 && OnstageCount() >= 2;
    }

    int FoulPoints() {
        return 2 * Fouls;
    }

    int TechFoulPoints() {
        return 5 * TechFouls;
    }

    int TotalScore() {
        return TotalAuto()
                + TotalTeleop()
                - FoulPoints()
                - TechFoulPoints()
                + Adjustments;
    }

    int RankingPoints() {
        int total = 0;
        if (MelodyBonus())
            total += 1;
        if (EnsembleBonus())
            total += 1;
        if (TotalScore() == opponent.TotalScore())
            total += 1;
        else if (TotalScore() > opponent.TotalScore())
            total += 2;
        return total;
    }

}
