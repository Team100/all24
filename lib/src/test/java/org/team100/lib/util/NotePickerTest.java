package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NotePickerTest {
    

    @Test
    void noNote() {
        assertEquals(NotePicker.autoNotePick(Optional.empty(),1), Optional.empty());
        assertEquals(NotePicker.closestNote(Optional.empty(),new Pose2d()), Optional.empty());
    }
    @Test
    void testObviousPick() {
        Translation2d[] e = {new Translation2d(), new Translation2d(10,10)};
        Translation2d expected = NotePicker.closestNote(Optional.of(e), new Pose2d()).get();
        Translation2d expected2 = NotePicker.closestNote(Optional.of(e), new Pose2d(new Translation2d(10,10), new Rotation2d())).get();
        assertEquals(expected, new Translation2d());
        assertEquals(expected2, new Translation2d(10,10));
    }
    @Test
    void testLessObviousPick() {
        Translation2d[] e = {new Translation2d(), new Translation2d(1,1)};
        Translation2d expected = NotePicker.closestNote(Optional.of(e), new Pose2d(new Translation2d(0.6,0.6), new Rotation2d())).get();
        assertEquals(expected, new Translation2d(1,1));
    }

    @Test
    void testAutoNotePickObvious() {
        Translation2d[] e = {new Translation2d(), new Translation2d(1,1), new Translation2d(2.9, 0.77)};
        Translation2d expected = NotePicker.autoNotePick(Optional.of(e), 1).get();
        assertEquals(expected, new Translation2d(2.9, 0.77));
    }
    
    @Test
    void testAutoNotePickLessObvious() {
        Translation2d[] e = {new Translation2d(), new Translation2d(2.8,0.77), new Translation2d(2.9, 0.77)};
        Translation2d expected = NotePicker.autoNotePick(Optional.of(e), 1).get();
        assertEquals(expected, new Translation2d(2.9, 0.77));
    }

    @Test
    void tsetFar() {
        Translation2d[] e = {new Translation2d(10,10), new Translation2d(41,41), new Translation2d(23.9, 0.377)};
        Optional<Translation2d> expected = NotePicker.autoNotePick(Optional.of(e), 1);
        assertEquals(expected, Optional.empty());
    }
}
