package org.team100.control.auto;

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
        ToSpeaker,
        ToSource
    }

    private enum Trigger {
        Done
    }

    private final StateMachineConfig<State, Trigger> config;

    private final StateMachine<State, Trigger> machine;

    public SpeakerCycler() {
        config = new StateMachineConfig<>();
        config.configure(State.ToSpeaker).permit(Trigger.Done, State.ToSource);
        config.configure(State.ToSource).permit(Trigger.Done, State.ToSpeaker);
        machine = new StateMachine<>(State.ToSource, config);
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
        return machine.isInState(State.ToSpeaker);
    }

    @Override
    public boolean driveToSource() {
        return machine.isInState(State.ToSource);
    }

    @Override
    public void onEnd() {
        machine.fire(Trigger.Done);
    }

}
