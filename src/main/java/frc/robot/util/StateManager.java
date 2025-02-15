package frc.robot.util;

import java.util.Optional;

public class StateManager<T> {
    private T currentState;
    private T wantedState;


    public StateManager(T currentState) {
        this.currentState = currentState;
    }

    public StateManager() {}

    public T getCurrentState() {
        return currentState;
    }

    public Optional<T> transitioningTo() {
        return Optional.ofNullable(wantedState);
    }

    public boolean isTransitioning() {
        return wantedState != null;
    }

    public void transitionTo(T wantedState) {
        this.wantedState = wantedState;
    }

    public void failTransition() {
        wantedState = null;
    }

    public void finishTransition() {
        currentState = wantedState;
        wantedState = null;
    }
}
