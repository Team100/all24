package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class NotePickerTest {
    private static final double kDelta = 0.0001;

    @Test
    void noNote() {
        // no sights == no notes
        assertTrue(NotePicker.autoNotePick(new ArrayList<>(), 1).isEmpty());
        Pose2d robotPose = new Pose2d();
        assertTrue(NotePicker.closestNote(new ArrayList<>(), robotPose).isEmpty());
    }

    @Test
    void testObviousPick() {
        List<Translation2d> sights = List.of(
                new Translation2d(),
                new Translation2d(10, 10));
        Pose2d robotPose = new Pose2d();
        Translation2d note = NotePicker.closestNote(sights, robotPose).get();
        // robot at origin -> closest note is at origin
        assertEquals(0, note.getX(), kDelta);
        assertEquals(0, note.getY(), kDelta);
    }

    @Test
    void testObviousPick2() {
        List<Translation2d> sights = List.of(
                new Translation2d(),
                new Translation2d(5, 5));
        Pose2d robotPose = new Pose2d(new Translation2d(5, 5), new Rotation2d());
        Translation2d note = NotePicker.closestNote(sights, robotPose).get();
        // choose the note close to the current pose of (5, 5)
        assertEquals(5, note.getX(), kDelta);
        assertEquals(5, note.getY(), kDelta);
    }

    @Test
    void testLessObviousPick() {
        List<Translation2d> sights = List.of(
                new Translation2d(),
                new Translation2d(1, 1));
        Pose2d robotPose = new Pose2d(new Translation2d(0.6, 0.6), new Rotation2d());
        Translation2d note = NotePicker.closestNote(sights, robotPose).get();
        assertEquals(1, note.getX(), kDelta);
        assertEquals(1, note.getY(), kDelta);
    }

    @Test
    void testAutoNotePickObvious() {
        List<Translation2d> sights = List.of(
                new Translation2d(),
                new Translation2d(1, 1),
                new Translation2d(2.9, 0.77));
        Translation2d note = NotePicker.autoNotePick(sights, 1).get();
        assertEquals(note, new Translation2d(2.9, 0.77));
        assertEquals(2.9, note.getX(), kDelta);
        assertEquals(0.77, note.getY(), kDelta);
    }

    @Test
    void testFar() {
        List<Translation2d> sights = List.of(
                new Translation2d(10, 10),
                new Translation2d(41, 41),
                new Translation2d(22.9, 0.77));
        Optional<Translation2d> note = NotePicker.autoNotePick(sights, 1);
        assertTrue(note.isEmpty());
    }
}
