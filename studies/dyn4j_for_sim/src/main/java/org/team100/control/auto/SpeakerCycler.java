package org.team100.control.auto;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

/**
 * Cycles between source and speaker.
 * 
 * @startuml
 *           state ToSource: Drive to the source
 *           state ToSpeaker: Drive to the speaker
 *           [*] -> ToSource
 *           ToSource -> ToSpeaker : Done
 *           ToSpeaker -> ToSource : Done
 * @enduml
 */
public class SpeakerCycler implements Autopilot {

    private enum State {
        Initial,
        ToSpeaker,
        ToSource
    }

    private enum Trigger {
        Begin,
        Done,
        Reset
    }

    private final StateMachine<State, Trigger> machine;

    public SpeakerCycler() {
        final StateMachineConfig<State, Trigger> config = new StateMachineConfig<>();
        config.configure(State.Initial)
                .permit(Trigger.Begin, State.ToSource)
                .ignore(Trigger.Done)
                .ignore(Trigger.Reset);
        config.configure(State.ToSpeaker)
                .ignore(Trigger.Begin)
                .permit(Trigger.Done, State.ToSource)
                .permit(Trigger.Reset, State.Initial);
        config.configure(State.ToSource)
                .ignore(Trigger.Begin)
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

    // @Override
    // public boolean intake() {
    // return false;
    // }

    // @Override
    // public boolean shoot() {
    // return false;
    // }

    // @Override
    // public boolean rotateToShoot() {
    // return false;
    // }

    // for now just drive back and forth



    @Override
    public boolean driveToSpeaker() {
        boolean inState = machine.isInState(State.ToSpeaker);
        // System.out.println("drive to speaker " + inState);
        return inState;
    }

    @Override
    public boolean driveToSource() {
        boolean inState = machine.isInState(State.ToSource);
        // System.out.println("drive to source " + inState);
        return inState;
    }

    @Override
    public void onEnd() {
        System.out.println("Cycler onEnd");
        machine.fire(Trigger.Done);
    }
    
    @Override
    public void periodic() {
        System.out.println("Cycler state: " + machine.getState());
    }
}
