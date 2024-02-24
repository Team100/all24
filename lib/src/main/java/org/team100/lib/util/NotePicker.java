package org.team100.lib.util;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NotePicker {

    // TODO Get real values here
    /**
     * This is the field relative position of every note on the field, going from
     * left buttom to right top
     */
    private static final Translation2d[] autoNotes = {
            new Translation2d(2.91, 0.76),
            new Translation2d(2.91, 2.41),
            new Translation2d(2.91, 4.11),
            new Translation2d(8.25, .76),
            new Translation2d(8.25, 2.41),
            new Translation2d(8.25, 4.11),
            new Translation2d(8.25, 5.78),
            new Translation2d(8.25, 7.47),
            new Translation2d(13.66, 0.76),
            new Translation2d(13.66, 2.41),
            new Translation2d(13.66, 4.11),
    };

    /**
     * @param notes  the field relative pose of detected notes
     * @param noteID the field id of the note you want to go for, left bottom to
     *               right top IDs are 1-11
     * @return The field relative translation of the note you want to go for
     */
    public static Optional<Translation2d> autoNotePick(Optional<ArrayList<Translation2d>> notes, int noteID) {
        if (!notes.isPresent()) {
            return Optional.empty();
        }
        ArrayList<Translation2d> Tnotes = notes.get();
        double bestNote = 1000000000;
        Optional<Translation2d> bestNoteTranslation = Optional.empty();
        for (Translation2d note : Tnotes) {
            Translation2d fieldNote = autoNotes[noteID - 1];
            Translation2d difference = note.minus(fieldNote);
            if (Math.abs(difference.getX()) < 1 && Math.abs(difference.getY()) < 1) {
                double average = Math.abs(difference.getX()) + Math.abs(difference.getY());
                if (average < bestNote) {
                    bestNote = average;
                    bestNoteTranslation = Optional.of(note);
                }
            }
        }
        return bestNoteTranslation;
    }

    /**
     * @param notes      the field relative pose of detected notes
     * @param swervePose the pose of the swerve drivetrain
     * @return The field relative translation of the closest note
     */
    public static Optional<Translation2d> closestNote(Optional<ArrayList<Translation2d>> notes, Pose2d swervePose) {
        if (!notes.isPresent()) {
            return Optional.empty();
        }
        ArrayList<Translation2d> Tnotes = notes.get();
        double bestNote = 1000000000;
        Optional<Translation2d> bestNoteTranslation = Optional.empty();
        for (Translation2d note : Tnotes) {
            double difference = Math.abs(note.minus(swervePose.getTranslation()).getNorm());
            if (difference < bestNote) {
                bestNote = difference;
                bestNoteTranslation = Optional.of(note);
            }
        }
        return bestNoteTranslation;
    }
}
