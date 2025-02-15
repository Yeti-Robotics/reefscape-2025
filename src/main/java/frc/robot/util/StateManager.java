package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Optional;

public class StateManager<T> {
    private T currentState;
    private T wantedState;
    private boolean transitionStarted = false;
    private final Subsystem subsystem;

    public StateManager(T currentState, Subsystem subsystem) {
        this.currentState = currentState;
        this.subsystem = subsystem;
    }

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
        subsystem.runOnce(() -> {
            this.wantedState = wantedState;
        });
    }

    public void startTransition() {
        transitionStarted = true;
    }

    public boolean transitionStarted() {
        return transitionStarted;
    }

    public void failTransition() {
        wantedState = null;
        transitionStarted = false;
    }

    public void finishTransition() {
        currentState = wantedState;
        wantedState = null;
        transitionStarted = false;
    }
}
