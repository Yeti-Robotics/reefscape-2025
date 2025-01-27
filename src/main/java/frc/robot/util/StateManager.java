package frc.robot.util;

public class StateManager<T> {
    private T currentState;
    private T targetState;

    public StateManager(T currentState) {
        this.currentState = currentState;
    }

    public StateManager() {}

    public T getState() {
        return targetState != null ? targetState : currentState;
    }

    public boolean isTransitioning() {
        return targetState != null;
    }

    public void transitionTo(T newTargetState) {
        if (newTargetState != currentState) {
            this.targetState = newTargetState;
        }
    }

    public void finishTransition() {
        currentState = targetState;
        targetState = null;
    }
}
