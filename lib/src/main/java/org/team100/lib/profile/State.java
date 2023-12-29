package org.team100.lib.profile;

import java.util.Objects;

/** This should be replaced by State100. */
public class State {
    public double position;

    public double velocity;

    public State() {
    }

    public State(double position, double velocity) {
        this.position = position;
        this.velocity = velocity;
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof State) {
            State rhs = (State) other;
            return this.position == rhs.position && this.velocity == rhs.velocity;
        } else {
            return false;
        }
    }

    @Override
    public int hashCode() {
        return Objects.hash(position, velocity);
    }
}