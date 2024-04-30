package org.team100.control.auto;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

/**
 * Cycles between source and amp.
 */
public class AmpCycler implements Autopilot {

    private enum State {
        Initial,
        ToAmp,
        ToSource
    }

    private enum Trigger {
        Begin,
        Done,
        Reset
    }

    private final StateMachine<State, Trigger> machine;

    public AmpCycler() {
        final StateMachineConfig<State, Trigger> config = new StateMachineConfig<>();
        config.configure(State.Initial)
                .permit(Trigger.Begin, State.ToSource);
        config.configure(State.ToAmp)
                .permit(Trigger.Done, State.ToSource)
                .permit(Trigger.Reset, State.Initial);
        config.configure(State.ToSource)
                .permit(Trigger.Done, State.ToAmp)
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
    public void onEnd() {
        System.out.println("Amp Cycler onEnd");
        machine.fire(Trigger.Done);
    }
    
    @Override
    public void periodic() {
        System.out.println("Amp Cycler state: " + machine.getState());
    }
}
