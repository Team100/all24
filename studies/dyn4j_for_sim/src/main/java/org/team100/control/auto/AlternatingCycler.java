package org.team100.control.auto;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

/**
 * Alternates between speaker and amp.
 * 
 * TODO: add alliance input for choosing
 * TODO: notice the amplified mode
 */
public class AlternatingCycler implements Autopilot {

    private enum State {
        Initial,
        ToSource,
        ToAmp,
        ToSpeaker
    }

    private enum Trigger {
        Begin,
        Done,
        Amp,
        Speaker,
        Reset
    }

    private final StateMachine<State, Trigger> machine;

    // placeholder for alliance strategy input or amplification input
    private boolean ampNext = false;

    public AlternatingCycler() {
        final StateMachineConfig<State, Trigger> config = new StateMachineConfig<>();
        config.configure(State.Initial)
                .permit(Trigger.Begin, State.ToSource);
        config.configure(State.ToSource)
                .permit(Trigger.Amp, State.ToAmp)
                .permit(Trigger.Speaker, State.ToSpeaker)
                .permit(Trigger.Reset, State.Initial);
        config.configure(State.ToAmp)
                .permit(Trigger.Done, State.ToSource)
                .permit(Trigger.Reset, State.Initial);
        config.configure(State.ToSpeaker)
                .permit(Trigger.Done, State.ToSource)
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
    public boolean driveToSource() {
        return machine.isInState(State.ToSource);
    }

    @Override
    public boolean driveToSpeaker() {
        return machine.isInState(State.ToSpeaker);
    }

    @Override
    public void onEnd() {
        System.out.println("Amp Cycler onEnd");
        if (machine.isInState(State.ToSource)) {
            if (ampNext) {
                machine.fire(Trigger.Amp);
            } else {
                machine.fire(Trigger.Speaker);
            }
            ampNext = !ampNext;
        } else {
            machine.fire(Trigger.Done);
        }
    }

    @Override
    public void periodic() {
        System.out.println("Amp Cycler state: " + machine.getState());
    }
}
