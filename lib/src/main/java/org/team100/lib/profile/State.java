package org.team100.lib.profile;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;

/** This should be replaced by State100. */
public class State {
    private double position;

    private double velocity;

    public State(double position, double velocity) {
        this.position = position;
        this.velocity = velocity;
    }

    public State(State x) {
        this.position = x.position;
        this.velocity = x.velocity;
    }

    public boolean near(State other, double tolerance) {
        return MathUtil.isNear(position, other.position, tolerance) &&
                MathUtil.isNear(velocity, other.velocity, tolerance);
    }

    public State minus(State other) {
        return new State(position - other.position, velocity - other.velocity);
    }

    public double norm() {
        return Math.hypot(position, velocity);
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

    @Override
    public String toString() {
        return String.format("State [position=%8.6f velocity=%8.6f]", position, velocity);
    }

    public double getPosition() {
        return position;
    }

    public void setPosition(double position) {
        this.position = position;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }
}