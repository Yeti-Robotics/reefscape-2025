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
        this.currentState = defaultState;
    }

    protected abstract StatusCode initializeTransition(T targetState);

    protected abstract boolean isTransitionFinished();

    protected void runPeriodic() {}

    protected void updateState(T targetState) {
        if (!transitionStarted) {
            transitionStarted = true;

            StatusCode code = initializeTransition(targetState);

            if (!code.isOK()) {
                failTransition();
                return;
            }
        }

        if (isTransitionFinished()) {
            finishTransition();
        }
    }

    @Override
    public final void periodic() {
        runPeriodic();

        if (isTransitioning()) {
            updateState(transitioningTo().orElse(defaultState));
        }
    }

    /**
     * Transition to the given state, or the default state supplied by the subsystem if the
     * transition is interrupted.
     *
     * @param state the target state to transition to.
     * @return a command that will transition to the given state, or the default state if the
     *     transition is interrupted.
     */
    public Command transitionTo(T state) {
        return transitionTo(
                state, defaultState); // most of the time, this fallback state will be HOLD, which
        // maintains the current position
    }

    /**
     * Transition to the given state, or fallback state if the transition is interrupted. Be careful
     * with this method, as it can lead to unexpected behavior if the fallback state is not
     * appropriate for the subsystem, it may cause sudden changes.
     *
     * @param state the target state to transition to.
     * @param fallbackState the fallback state to use if the transition is interrupted.
     * @return A command that will transition to the given state, or fallback state if the
     *     transition is interrupted.
     * @see StatefulSubsystem#transitionTo(Enum) if you don't want to specify a fallback state
     */
    public Command transitionTo(T state, T fallbackState) {
        Runnable finishOrFail =
                () -> {
                    if (!isTransitionFinished()) {
                        failTransition();
                    } else {
                        finishTransition();
                    }

                    if (currentState == null) {
                        currentState = defaultState;
                    }
                };

        return startEnd(() -> transitionToState(state), () -> {})
                .until(() -> !isTransitioning())
                .handleInterrupt(finishOrFail);
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
        if (isTransitioning()) {
            failTransition();
        }

        wantedState = state;
    }

    protected void failTransition() {
        wantedState = null;
        transitionStarted = false;
    }

    private void finishTransition() {
        currentState = wantedState;
        wantedState = null;
        transitionStarted = false;
    }
}
