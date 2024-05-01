package org.team100.control.auto;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

/**
 * Scorer goes back and forth from speaker to amp
 * TODO: add drive-to-note to pick up and score.
 */
public class Scorer implements Autopilot {

    private enum State {
        Initial,
        ToSpeaker,
        ToAmp
    }

    private enum Trigger {
        Begin,
        Done,
        Reset
    }

    private final StateMachine<State, Trigger> machine;

    public Scorer() {
        final StateMachineConfig<State, Trigger> config = new StateMachineConfig<>();
        config.configure(State.Initial)
                .permit(Trigger.Begin, State.ToSpeaker);
        config.configure(State.ToSpeaker)
                .permit(Trigger.Done, State.ToAmp)
                .permit(Trigger.Reset, State.Initial);
        config.configure(State.ToAmp)
                .permit(Trigger.Done, State.ToSpeaker)
                .permit(Trigger.Reset, State.Initial);
        try {
            ByteArrayOutputStream dotFile = new ByteArrayOutputStream();
            config.generateDotFileInto(dotFile);
            String actual = new String(dotFile.toByteArray(), StandardCharsets.UTF_8);
            System.out.println(actual);
        } catch (IOException e) {
            e.printStackTrace();
        }
        machine = new StateMachine<>(State.Initial, config);
        machine.onUnhandledTrigger((s, t) -> {
        });
        machine.fireInitialTransition();
    }

    @Override
    public void begin() {
        machine.fire(Trigger.Begin);
    }

    @Override
    public void reset() {
        machine.fire(Trigger.Reset);
    }

    @Override
    public boolean driveToAmp() {
        return machine.isInState(State.ToAmp);
    }

    @Override
    public boolean driveToSpeaker() {
        return machine.isInState(State.ToSpeaker);
    }

    @Override
    public void onEnd() {
        machine.fire(Trigger.Done);
    }

    @Override
    public void periodic() {
    }

}
