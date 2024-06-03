package org.team100.control.auto;

import org.team100.control.Pilot;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;

/** Fetch a note, shoot it into the speaker, repeat. */
public class Auton implements Pilot {

    private final IndexerSubsystem m_indexer;
    private final Integer[] notes;

    private boolean m_enabled = false;

    public Auton(IndexerSubsystem indexer, Integer... note) {
        Arg.nonnull(indexer);
        Arg.nonempty(note);
        notes = note;
        m_indexer = indexer;
    }

    @Override
    public void begin() {
        m_enabled = true;
    }

    @Override
    public void reset() {
        m_enabled = false;
    }

    @Override
    public boolean scoreSpeaker() {
        return m_enabled && m_indexer.full();
    }

    @Override
    public int goalNote() {
        return notes[0];
    }

}
