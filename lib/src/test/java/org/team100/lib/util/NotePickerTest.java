package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class NotePickerTest {

    @Test
    void noNote() {
        assertEquals(NotePicker.autoNotePick(new ArrayList<>(), 1), Optional.empty());
        assertEquals(NotePicker.closestNote(new ArrayList<>(), new Pose2d()), Optional.empty());
    }

    @Test
    void testObviousPick() {
        List<Translation2d> e = new ArrayList<>();
        e.add(new Translation2d());
        e.add(new Translation2d(10, 10));
        Translation2d expected = NotePicker.closestNote(e, new Pose2d()).get();
        Translation2d expected2 = NotePicker.closestNote(e,
                new Pose2d(new Translation2d(10, 10), new Rotation2d())).get();
        assertEquals(expected, new Translation2d());
        assertEquals(expected2, new Translation2d(10, 10));
    }

    @Test
    void testLessObviousPick() {
        List<Translation2d> e = new ArrayList<>();
        e.add(new Translation2d());
        e.add(new Translation2d(1, 1));
        Translation2d expected = NotePicker.closestNote(e,
                new Pose2d(new Translation2d(0.6, 0.6), new Rotation2d())).get();
        assertEquals(expected, new Translation2d(1, 1));
    }

    @Test
    void testAutoNotePickObvious() {
        List<Translation2d> e = new ArrayList<>();
        e.add(new Translation2d());
        e.add(new Translation2d(1, 1));
        e.add(new Translation2d(2.9, 0.77));
        Translation2d expected = NotePicker.autoNotePick(e, 1).get();
        assertEquals(expected, new Translation2d(2.9, 0.77));
    }

    @Test
    void testAutoNotePickLessObvious() {
        List<Translation2d> e = new ArrayList<>();
        e.add(new Translation2d());
        e.add(new Translation2d(1, 1));
        e.add(new Translation2d(2.9, 0.77));
        Translation2d expected = NotePicker.autoNotePick(e, 1).get();
        assertEquals(expected, new Translation2d(2.9, 0.77));
    }

    @Test
    void testFar() {
        List<Translation2d> e = new ArrayList<>();
        e.add(new Translation2d(10, 10));
        e.add(new Translation2d(41, 41));
        e.add(new Translation2d(22.9, 0.77));
        Optional<Translation2d> expected = NotePicker.autoNotePick(e, 1);
        assertEquals(expected, Optional.empty());
    }
}
