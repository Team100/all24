package org.team100.lib.util;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NotePicker {

    /**
     * @param notes     the field relative pose of detected notes
     * @param robotPose the pose of the swerve drivetrain
     * @return The field relative translation of the closest note, or empty if none
     *         nearby
     */
    public static Optional<Translation2d> closestNote(
            List<Translation2d> notes,
            Pose2d robotPose) {
        if (notes.isEmpty()) {
            return Optional.empty();
        }
        double bestNote = 1000000000;
        Optional<Translation2d> bestNoteTranslation = Optional.empty();
        for (Translation2d note : notes) {
            // if (note.getY() < -1 || note.getX() < -1 || note.getY() > 9.21 || note.getX() > 17.54) {
            //     // ignore out-of-bounds
            //     System.out.println()
            //     continue;
            // }
            double difference = Math.abs(note.minus(robotPose.getTranslation()).getNorm());
            if (difference < bestNote) {
                bestNote = difference;
                bestNoteTranslation = Optional.of(note);
            }
        }
        return bestNoteTranslation;
    }

    private NotePicker() {
        //
    }
}
