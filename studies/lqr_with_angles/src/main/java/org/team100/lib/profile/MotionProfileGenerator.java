package org.team100.lib.profile;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;

import org.team100.lib.util.DoubleProgression;
import org.team100.lib.util.MathUtil;

import edu.wpi.first.math.Pair;

/**
 * Motion profile generator with arbitrary start and end motion states and
 * either dynamic constraints or jerk limiting.
 */
public class MotionProfileGenerator {

    private static class EvaluatedConstraint {
        double maxVel;
        double maxAccel;

        public EvaluatedConstraint(double maxVel, double maxAccel) {
            this.maxVel = maxVel;
            this.maxAccel = maxAccel;
        }

    }

    /**
     * Generates a simple motion profile with constant [maxVel], [maxAccel], and
     * [maxJerk]. If [maxJerk] is zero, an
     * acceleration-limited profile will be generated instead of a jerk-limited one.
     * If constraints can't be obeyed,
     * there are two possible fallbacks: If [overshoot] is true, then two profiles
     * will be concatenated (the first one
     * overshoots the goal and the second one reverses back to reach the goal).
     * Otherwise, the highest order constraint
     * (e.g., max jerk for jerk-limited profiles) is repeatedly violated until the
     * goal is achieved.
     *
     * @param start     start motion state
     * @param goal      goal motion state
     * @param maxVel    maximum velocity
     * @param maxAccel  maximum acceleration
     * @param maxJerk   maximum jerk
     * @param overshoot if true overshoot otherwise violate constraints (see
     *                  description above)
     */

    public static MotionProfile generateSimpleMotionProfile(
            MotionState start,
            MotionState goal,
            double maxVel,
            double maxAccel,
            boolean overshoot) {
        return generateSimpleMotionProfile(start, goal, maxVel, maxAccel, 0.0, overshoot);
    }

    public static MotionProfile generateSimpleMotionProfile(
            MotionState start,
            MotionState goal,
            double maxVel,
            double maxAccel) {
        return generateSimpleMotionProfile(start, goal, maxVel, maxAccel, 0.0);
    }

    public static MotionProfile generateSimpleMotionProfile(
            MotionState start,
            MotionState goal,
            double maxVel,
            double maxAccel,
            double maxJerk) {
        return generateSimpleMotionProfile(start, goal, maxVel, maxAccel, maxJerk, false);
    }

    public static MotionProfile generateSimpleMotionProfile(
            MotionState start,
            MotionState goal,
            double maxVel,
            double maxAccel,
            double maxJerk,
            boolean overshoot) {
        // ensure the goal is always after the start; plan the flipped profile otherwise
        if (goal.getX() < start.getX()) {
            return generateSimpleMotionProfile(
                    start.flipped(),
                    goal.flipped(),
                    maxVel,
                    maxAccel,
                    maxJerk).flipped();
        }

        if (MathUtil.epsilonEquals(maxJerk, 0.0)) {
            // acceleration-limited profile (trapezoidal)
            double requiredAccel = (goal.getV() * goal.getV() - start.getV() * start.getV())
                    / (2 * (goal.getX() - start.getX()));

            MotionProfile accelProfile = generateAccelProfile(start, maxVel, maxAccel);
            MotionProfile decelProfile = generateAccelProfile(
                    new MotionState(
                            goal.getX(),
                            goal.getV(),
                            -goal.getA(),
                            goal.getJ()),
                    maxVel,
                    maxAccel,
                    maxJerk)
                    .reversed();

            MotionProfile noCoastProfile = accelProfile.plus(decelProfile);
            double remainingDistance = goal.getX() - noCoastProfile.end().getX();

            if (remainingDistance >= 0.0) {
                // normal 3-segment profile works
                double deltaT2 = remainingDistance / maxVel;

                return new MotionProfileBuilder(start)
                        .appendProfile(accelProfile)
                        .appendAccelerationControl(0.0, deltaT2)
                        .appendProfile(decelProfile)
                        .build();
            } else if (Math.abs(requiredAccel) > maxAccel) {
                if (overshoot) {
                    // TODO: is this most efficient? (do we care?)
                    return noCoastProfile.plus(generateSimpleMotionProfile(
                            noCoastProfile.end(),
                            goal,
                            maxVel,
                            maxAccel,
                            overshoot = true));
                } else {
                    // single segment profile
                    double dt = (goal.getV() - start.getV()) / requiredAccel;
                    return new MotionProfileBuilder(start)
                            .appendAccelerationControl(requiredAccel, dt)
                            .build();
                }
            } else if (start.getV() > maxVel && goal.getV() > maxVel) {
                // decel, accel
                List<Double> roots = MathUtil.solveQuadratic(
                        -maxAccel,
                        2 * start.getV(),
                        (goal.getV() * goal.getV() - start.getV() * start.getV()) / (2 * maxAccel) - goal.getX()
                                + start.getX());
                double deltaT1 = roots.stream().filter((it) -> it >= 0.0).min(Double::compare).orElseThrow();
                double deltaT3 = Math.abs(start.getV() - goal.getV()) / maxAccel + deltaT1;

                return new MotionProfileBuilder(start)
                        .appendAccelerationControl(-maxAccel, deltaT1)
                        .appendAccelerationControl(maxAccel, deltaT3)
                        .build();
            } else {
                // accel, decel
                List<Double> roots = MathUtil.solveQuadratic(
                        maxAccel,
                        2 * start.getV(),
                        (start.getV() * start.getV() - goal.getV() * goal.getV()) / (2 * maxAccel) - goal.getX()
                                + start.getX());
                double deltaT1 = roots.stream().filter((it) -> it >= 0.0).min(Double::compare).orElseThrow();
                double deltaT3 = Math.abs(start.getV() - goal.getV()) / maxAccel + deltaT1;

                return new MotionProfileBuilder(start)
                        .appendAccelerationControl(maxAccel, deltaT1)
                        .appendAccelerationControl(-maxAccel, deltaT3)
                        .build();
            }
        } else {
            // jerk-limited profile (S-curve)
            MotionProfile accelerationProfile = generateAccelProfile(start, maxVel, maxAccel, maxJerk);
            // we leverage symmetry here; deceleration profiles are just reversed
            // acceleration ones with the goal
            // acceleration flipped
            MotionProfile decelerationProfile = generateAccelProfile(
                    new MotionState(
                            goal.getX(),
                            goal.getV(),
                            -goal.getA(),
                            goal.getJ()),
                    maxVel,
                    maxAccel,
                    maxJerk)
                    .reversed();

            MotionProfile noCoastProfile = accelerationProfile.plus(decelerationProfile);
            double remainingDistance = goal.getX() - noCoastProfile.end().getX();

            if (remainingDistance >= 0.0) {
                // we just need to add a coast segment of appropriate duration
                double deltaT4 = remainingDistance / maxVel;

                return new MotionProfileBuilder(start)
                        .appendProfile(accelerationProfile)
                        .appendJerkControl(0.0, deltaT4)
                        .appendProfile(decelerationProfile)
                        .build();
            } else {
                // the profile never reaches maxV
                // thus, we need to compute the peak velocity (0 < peak vel < max vel)
                // we *could* construct a large polynomial expression (i.e., a nasty cubic) and
                // solve it using Cardano's
                // method, some kind of inclusion method like modified Anderson-Bjorck-King, or
                // a host of other methods
                // (see https://link.springer.com/content/pdf/bbm%3A978-3-642-05175-3%2F1.pdf
                // for modified ABK)
                // instead, however, we conduct a binary search as it's sufficiently performant
                // for this use case,
                // requires less code, and is overall significantly more comprehensible
                double upperBound = maxVel;
                double lowerBound = 0.0;
                int iterations = 0;
                while (iterations < 1000) {
                    double peakVel = (upperBound + lowerBound) / 2;

                    MotionProfile searchAccelProfile = generateAccelProfile(start, peakVel, maxAccel, maxJerk);
                    MotionProfile searchDecelProfile = generateAccelProfile(goal, peakVel, maxAccel, maxJerk)
                            .reversed();

                    MotionProfile searchProfile = searchAccelProfile.plus(searchDecelProfile);

                    double error = goal.getX() - searchProfile.end().getX();

                    if (MathUtil.epsilonEquals(error, 0.0)) {
                        return searchProfile;
                    }

                    if (error > 0.0) {
                        // we undershot so shift the lower bound up
                        lowerBound = peakVel;
                    } else {
                        // we overshot so shift the upper bound down
                        upperBound = peakVel;
                    }

                    iterations++;
                }

                // constraints are not satisfiable
                if (overshoot) {
                    return noCoastProfile.plus(generateSimpleMotionProfile(
                            noCoastProfile.end(),
                            goal,
                            maxVel,
                            maxAccel,
                            maxJerk,
                            overshoot = true));
                } else {
                    // violate max jerk first
                    return generateSimpleMotionProfile(
                            start,
                            goal,
                            maxVel,
                            maxAccel,
                            overshoot = false);
                }
            }
        }
    }

    private static MotionProfile generateAccelProfile(
            MotionState start,
            double maxVel,
            double maxAccel) {
        return generateAccelProfile(
                start, maxVel, maxAccel, 0.0);
    }

    private static MotionProfile generateAccelProfile(
            MotionState start,
            double maxVel,
            double maxAccel,
            double maxJerk) {
        if (MathUtil.epsilonEquals(maxJerk, 0.0)) {
            // acceleration-limited
            double deltaT1 = Math.abs(start.getV() - maxVel) / maxAccel;
            MotionProfileBuilder builder = new MotionProfileBuilder(start);
            if (start.getV() > maxVel) {
                // we need to decelerate
                builder.appendAccelerationControl(-maxAccel, deltaT1);
            } else {
                builder.appendAccelerationControl(maxAccel, deltaT1);
            }
            return builder.build();
        } else {
            // jerk-limited
            // compute the duration and velocity of the first segment
            double deltaT1;
            double deltaV1;
            if (start.getA() > maxAccel) {
                // slow down and see where we are
                deltaT1 = (start.getA() - maxAccel) / maxJerk;
                deltaV1 = start.getA() * deltaT1 - 0.5 * maxJerk * deltaT1 * deltaT1;
            } else {
                // otherwise accelerate
                deltaT1 = (maxAccel - start.getA()) / maxJerk;
                deltaV1 = start.getA() * deltaT1 + 0.5 * maxJerk * deltaT1 * deltaT1;
            }

            // compute the duration and velocity of the third segment
            double deltaT3 = maxAccel / maxJerk;
            double deltaV3 = maxAccel * deltaT3 - 0.5 * maxJerk * deltaT3 * deltaT3;

            // compute the velocity change required in the second segment
            double deltaV2 = maxVel - start.getV() - deltaV1 - deltaV3;

            if (deltaV2 < 0.0) {
                // there is no constant acceleration phase
                // the second case checks if we're going to exceed max vel
                if (start.getA() > maxAccel
                        || (start.getV() - maxVel) > (start.getA() * start.getA()) / (2 * maxJerk)) {
                    // problem: we need to cut down on our acceleration but we can't cut our initial
                    // decel
                    // solution: we'll lengthen our initial decel to -max accel and similarly with
                    // our final accel
                    // if this results in an over correction, decel instead to a good accel
                    double newDeltaT1 = (start.getA() + maxAccel) / maxJerk;
                    double newDeltaV1 = start.getA() * newDeltaT1 - 0.5 * maxJerk * newDeltaT1 * newDeltaT1;

                    double newDeltaV2 = maxVel - start.getV() - newDeltaV1 + deltaV3;

                    if (newDeltaV2 > 0.0) {
                        // we decelerated too much
                        List<Double> roots = MathUtil.solveQuadratic(
                                -maxJerk,
                                2 * start.getA(),
                                start.getV() - maxVel - start.getA() * start.getA() / (2 * maxJerk));
                        double finalDeltaT1 = roots.stream().filter((it) -> it >= 0.0).min(Double::compare)
                                .orElseThrow();
                        double finalDeltaT3 = finalDeltaT1 - start.getA() / maxJerk;

                        return new MotionProfileBuilder(start)
                                .appendJerkControl(-maxJerk, finalDeltaT1)
                                .appendJerkControl(maxJerk, finalDeltaT3)
                                .build();
                    } else {
                        // we're almost good
                        double newDeltaT2 = newDeltaV2 / -maxAccel;

                        return new MotionProfileBuilder(start)
                                .appendJerkControl(-maxJerk, newDeltaT1)
                                .appendJerkControl(0.0, newDeltaT2)
                                .appendJerkControl(maxJerk, deltaT3)
                                .build();
                    }
                } else {
                    // cut out the constant accel phase and find a shorter delta t1 and delta t3
                    List<Double> roots = MathUtil.solveQuadratic(
                            maxJerk,
                            2 * start.getA(),
                            start.getV() - maxVel + start.getA() * start.getA() / (2 * maxJerk));
                    double newDeltaT1 = roots.stream().filter((it) -> it >= 0.0).min(Double::compare).orElseThrow();
                    double newDeltaT3 = newDeltaT1 + start.getA() / maxJerk;

                    return new MotionProfileBuilder(start)
                            .appendJerkControl(maxJerk, newDeltaT1)
                            .appendJerkControl(-maxJerk, newDeltaT3)
                            .build();
                }
            } else {
                // there is a constant acceleration phase
                double deltaT2 = deltaV2 / maxAccel;

                MotionProfileBuilder builder = new MotionProfileBuilder(start);
                if (start.getA() > maxAccel) {
                    builder.appendJerkControl(-maxJerk, deltaT1);
                } else {
                    builder.appendJerkControl(maxJerk, deltaT1);
                }
                return builder.appendJerkControl(0.0, deltaT2)
                        .appendJerkControl(-maxJerk, deltaT3)
                        .build();
            }
        }
    }

    /**
     * Generates a motion profile with dynamic maximum velocity and acceleration.
     * Uses the algorithm described in
     * section 3.2 of
     * [Sprunk2008.pdf](http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf).
     * Warning:
     * Profiles may be generated incorrectly if the endpoint velocity/acceleration
     * values preclude the obedience of the
     * motion constraints. To protect against this, verify the continuity of the
     * generated profile or keep the start and
     * goal velocities at 0.
     *
     * @param start                  start motion state
     * @param goal                   goal motion state
     * @param velocityConstraint     velocity constraint
     * @param accelerationConstraint acceleration constraint
     * @param resolution             separation between constraint samples
     */
    public static MotionProfile generateMotionProfile(MotionState start,
            MotionState goal,
            VelocityConstraint velocityConstraint,
            AccelerationConstraint accelerationConstraint) {
        return generateMotionProfile(start, goal, velocityConstraint, accelerationConstraint, 0.25);
    }

    public static MotionProfile generateMotionProfile(
            MotionState start,
            MotionState goal,
            VelocityConstraint velocityConstraint,
            AccelerationConstraint accelerationConstraint,
            double resolution) {
        if (goal.getX() < start.getX()) {
            return generateMotionProfile(
                    start.flipped(),
                    goal.flipped(),
                    new VelocityConstraint() {
                        public double get(double s) {
                            return velocityConstraint.get(-s);
                        }
                    },
                    new AccelerationConstraint() {
                        public double get(double s) {
                            return accelerationConstraint.get(-s);
                        }
                    },
                    resolution).flipped();
        }

        double length = goal.getX() - start.getX();
        // dx is an adjusted resolution that fits nicely within length
        // at least two samples are required to have a valid profile
        int samples = Math.max(2, (int) Math.ceil(length / resolution));

        DoubleProgression s = DoubleProgression.fromClosedInterval(0.0, length, samples);
        List<EvaluatedConstraint> constraintsList = StreamSupport.stream(s.plus(start.getX()).spliterator(), false)
                .map((it) -> new EvaluatedConstraint(
                        velocityConstraint.get(it),
                        accelerationConstraint.get(it)))
                .collect(Collectors.toList());

        // compute the forward states
        List<Pair<MotionState, Double>> forwardStates = forwardPass(
                new MotionState(0.0, start.getV(), start.getA()),
                s,
                constraintsList).stream().map((it) -> {
                    MotionState motionState = it.getFirst();
                    double dx = it.getSecond();
                    return new Pair<>(new MotionState(
                            motionState.getX() + start.getX(),
                            motionState.getV(),
                            motionState.getA()), dx);
                }).collect(Collectors.toList());

        // compute the backward states
        List<EvaluatedConstraint> backwardsConstraints = new ArrayList<EvaluatedConstraint>(constraintsList);
        Collections.reverse(backwardsConstraints);
        List<Pair<MotionState, Double>> backwardStates = forwardPass(
                new MotionState(0.0, goal.getV(), goal.getA()),
                s,
                backwardsConstraints).stream().map((it) -> {
                    MotionState motionState = it.getFirst();
                    double dx = it.getSecond();
                    return new Pair<>(afterDisplacement(motionState, dx), dx);
                }).map((it) -> {
                    MotionState motionState = it.getFirst();
                    double dx = it.getSecond();
                    return new Pair<>(
                            new MotionState(
                                    goal.getX() - motionState.getX(),
                                    motionState.getV(),
                                    -motionState.getA()),
                            dx);
                }).collect(Collectors.toList());
        Collections.reverse(backwardStates);

        // merge the forward and backward states
        List<Pair<MotionState, Double>> finalStates = new ArrayList<Pair<MotionState, Double>>();

        int i = 0;
        while (i < forwardStates.size() && i < backwardStates.size()) {
            // retrieve the start states and displacement deltas
            MotionState forwardStartState = forwardStates.get(i).getFirst();
            double forwardDx = forwardStates.get(i).getSecond();
            MotionState backwardStartState = backwardStates.get(i).getFirst();
            double backwardDx = backwardStates.get(i).getSecond();

            // if there's a discrepancy in the displacements, split the the longer chunk in
            // two and add the second
            // to the corresponding list; this guarantees that segments are always aligned
            if (!(MathUtil.epsilonEquals(forwardDx, backwardDx))) {
                if (forwardDx > backwardDx) {
                    // forward longer
                    forwardStates.add(
                            i + 1,
                            new Pair<>(afterDisplacement(forwardStartState, backwardDx), forwardDx - backwardDx));
                    forwardDx = backwardDx;
                } else {
                    // backward longer
                    backwardStates.add(
                            i + 1,
                            new Pair<>(afterDisplacement(backwardStartState, forwardDx), backwardDx - forwardDx));
                    backwardDx = forwardDx;
                }
            }

            // compute the end states (after alignment)
            MotionState forwardEndState = afterDisplacement(forwardStartState, forwardDx);
            MotionState backwardEndState = afterDisplacement(backwardStartState, backwardDx);

            if (forwardStartState.getV() <= backwardStartState.getV()) {
                // forward start lower
                if (forwardEndState.getV() <= backwardEndState.getV()) {
                    // forward end lower
                    finalStates.add(new Pair<>(forwardStartState, forwardDx));
                } else {
                    // backward end lower
                    double intersection = intersection(
                            forwardStartState,
                            backwardStartState);
                    finalStates.add(new Pair<>(forwardStartState, intersection));
                    finalStates.add(
                            new Pair<>(
                                    afterDisplacement(backwardStartState, intersection),
                                    backwardDx - intersection));
                }
            } else {
                // backward start lower
                if (forwardEndState.getV() >= backwardEndState.getV()) {
                    // backward end lower
                    finalStates.add(new Pair<>(backwardStartState, backwardDx));
                } else {
                    // forward end lower
                    double intersection = intersection(
                            forwardStartState,
                            backwardStartState);
                    finalStates.add(new Pair<>(backwardStartState, intersection));
                    finalStates.add(
                            new Pair<>(
                                    afterDisplacement(forwardStartState, intersection),
                                    forwardDx - intersection));
                }
            }
            i++;
        }

        // turn the final states into actual time-parameterized motion segments
        List<MotionSegment> motionSegments = new ArrayList<MotionSegment>();
        for (Pair<MotionState, Double> finalState : finalStates) {
            MotionState state = finalState.getFirst();
            double stateDx = finalState.getSecond();
            double dt;
            if (MathUtil.epsilonEquals(state.getA(), 0.0)) {
                dt = stateDx / state.getV();
            } else {
                double discriminant = state.getV() * state.getV() + 2 * state.getA() * stateDx;
                if (MathUtil.epsilonEquals(discriminant, 0.0)) {
                    dt = -state.getV() / state.getA();
                } else {
                    dt = (Math.sqrt(discriminant) - state.getV()) / state.getA();
                }
            }
            motionSegments.add(new MotionSegment(state, dt));
        }

        return new MotionProfile(motionSegments);
    }

    // execute a forward pass that consists of applying maximum acceleration
    // starting at min(last velocity, max vel)
    // on a segment-by-segment basis
    private static List<Pair<MotionState, Double>> forwardPass(
            MotionState start,
            DoubleProgression displacements,
            List<EvaluatedConstraint> constraints) {
        List<Pair<MotionState, Double>> forwardStates = new ArrayList<Pair<MotionState, Double>>();

        double dx = displacements.getStep();

        MotionState lastState = start;
        int lengths = Math.min(displacements.size(), constraints.size());
        for (int i = 0; i < lengths; ++i) {
            double displacement = displacements.get(i);
            EvaluatedConstraint constraint = constraints.get(i);
            // compute the segment constraints
            double maxVel = constraint.maxVel;
            double maxAccel = constraint.maxAccel;

            if (lastState.getV() >= maxVel) {
                // the last velocity exceeds max vel so we just coast
                MotionState state = new MotionState(displacement, maxVel, 0.0);
                forwardStates.add(new Pair<>(state, dx));
                lastState = afterDisplacement(state, dx);
            } else {
                // compute the final velocity assuming max accel
                double finalVel = Math.sqrt(lastState.getV() * lastState.getV() + 2 * maxAccel * dx);
                if (finalVel <= maxVel) {
                    // we're still under max vel so we're good
                    MotionState state = new MotionState(displacement, lastState.getV(), maxAccel);
                    forwardStates.add(new Pair<>(state, dx));
                    lastState = afterDisplacement(state, dx);
                } else {
                    // we went over max vel so now we split the segment
                    double accelDx = (maxVel * maxVel - lastState.getV() * lastState.getV()) / (2 * maxAccel);
                    MotionState accelState = new MotionState(displacement, lastState.getV(), maxAccel);
                    MotionState coastState = new MotionState(displacement + accelDx, maxVel, 0.0);
                    forwardStates.add(new Pair<>(accelState, accelDx));
                    forwardStates.add(new Pair<>(coastState, dx - accelDx));
                    lastState = afterDisplacement(coastState, dx - accelDx);
                }
            }
        }

        return forwardStates;
    }

    private static MotionState afterDisplacement(MotionState state, double dx) {
        double discriminant = state.getV() * state.getV() + 2 * state.getA() * dx;
        if (MathUtil.epsilonEquals(discriminant, 0.0)) {
            return new MotionState(state.getX() + dx, 0.0, state.getA());
        } else {
            return new MotionState(state.getX() + dx, Math.sqrt(discriminant), state.getA());
        }
    }

    private static double intersection(MotionState state1, MotionState state2) {
        return (state1.getV() * state1.getV() - state2.getV() * state2.getV())
                / (2 * state2.getA() - 2 * state1.getA());
    }

}
