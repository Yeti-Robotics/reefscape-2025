package frc.robot.util.state.target;

import java.util.function.Supplier;

public class DynamicStateProvider<T> extends StateProvider<T> {
    private final SettableStateTarget<T> fallbackState;

    public DynamicStateProvider(Supplier<T> stateSupplier) {
        fallbackState = new SettableStateTarget<>(stateSupplier);
    }

    @Override
    protected StateTarget<T> findClosest(T value) {
        StateTarget<T> s = super.findClosest(value);

        return s == null ? fallbackState : s;
    }
}
