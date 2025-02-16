package frc.robot.util.state;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public abstract class StatefulSubsystem<T extends Enum<T>> extends SubsystemBase {
    private T currentState;
    private T wantedState;
    private final T defaultState;
    private boolean transitionStarted = false;

    public StatefulSubsystem(T defaultState) {
        this.defaultState = defaultState;
    }

    public abstract StatusCode initializeTransition(T targetState);

    public abstract boolean checkTransitionFinished();

    public void runPeriodic() {
        // no-op
    }

    protected void updateState(T targetState) {
        if (!transitionStarted) {
            transitionStarted = true;

            StatusCode code = initializeTransition(targetState);

            if (!code.isOK()) {
                failTransition();
                return;
            }
        }

        if (checkTransitionFinished()) {
            finishTransition();
        }
    }

    @Override
    final public void periodic() {
        runPeriodic();

        if (isTransitioning()) {
            updateState(transitioningTo().orElse(defaultState));
        }
    }

    public Command transitionTo(T state) {
        return runOnce(() -> transitionToState(state));
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

    protected void transitionToState(T state) {
        wantedState = state;
    }

    private void failTransition() {
        wantedState = null;
        transitionStarted = false;
    }

    private void finishTransition() {
        currentState = wantedState;
        wantedState = null;
        transitionStarted = false;
    }
}
