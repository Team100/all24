package org.team100.field;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * These locations match the background image, ~2 cm different from CAD.
 */
public enum StagedNote {
    NOTE1(1, 2.890, 4.10),
    NOTE2(2, 2.890, 5.56),
    NOTE3(3, 2.890, 7.01),

    NOTE4(4, 8.275, 0.75),
    NOTE5(5, 8.275, 2.43),
    NOTE6(6, 8.275, 4.10),
    NOTE7(7, 8.275, 5.79),
    NOTE8(8, 8.275, 7.47),

    NOTE9(9, 13.657, 4.10),
    NOTE10(10, 13.657, 5.56),
    NOTE11(11, 13.657, 7.01);

    private static final Map<Integer, StagedNote> stagedNotes = new HashMap<>();

    static {
        for (StagedNote n : StagedNote.values()) {
            stagedNotes.put(n.m_id, n);
        }
    }

    public static Optional<StagedNote> get(int id) {
        if (stagedNotes.containsKey(id)) {
            return Optional.of(stagedNotes.get(id));
        }
        return Optional.empty();
    }

    private final int m_id;
    private final Translation2d m_location;

    private StagedNote(int id, double x, double y) {
        m_id = id;
        m_location = new Translation2d(x, y);
    }

    public Translation2d getLocation() {
        return m_location;
    }

}
