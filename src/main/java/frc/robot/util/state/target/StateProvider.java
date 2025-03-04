package frc.robot.util.state.target;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class StateProvider<T> {
    private final List<StateTarget<T>> states = new ArrayList<>();

    protected StateProvider() {}

    public StateTarget<T> of(T value) {
        StateTarget<T> stateTarget = new ConstStateTarget<>(value);
        states.add(stateTarget);
        return stateTarget;
    }

    public  <Z extends StateTarget<T>> void ofAll(Z ...stateTarget) {
        Collections.addAll(states, stateTarget);
    }

    protected StateTarget<T> findClosest(T value) {
        for (StateTarget<T> state : states) {
            if (state.equals(value)) {
                return state;
            }
        }

        return null;
    }
}
